function determinant(a,b,c,d){
  return a*d - b*c;
}

function intersect_edges(a1,a2,b1,b2){
  // s a1x + (1-s) a2x = t b1x + (1-t) a2x
  var a = a1[0];
  var b = a2[0]-a1[0];
  var c = b1[0];
  var d = b2[0]-b1[0];
  var e = a1[1];
  var f = a2[1]-a1[1];
  var g = b1[1];
  var h = b2[1]-b1[1];

  var det = determinant(b,-d,f,-h);
  var a_time = determinant(c-a,-d,g-e,-h)/det;
  var b_time = determinant(b,c-a,f,g-e)/det;

  return {a:a_time,b:b_time};
}

function move_pos(start,vel,t){
    return numeric.add(start, numeric.mul(vel,t));
}

function vector_angle_old(x1,y1,x2,y2){
  var dot = x1*x2 + y1*y2;
  var cross_z = x1*y2 - y1*x2;
  var angle = Math.atan2(cross_z,dot);
  return angle;
}

function vector_angle(a,b){
  var dot = a[0]*b[0] + a[1]*b[1];
  var cross_z = a[0]*b[1] - a[1]*b[0];
  var angle = Math.atan2(cross_z,dot);
  return angle;
}

function rotate(vec, angle){
    return [
        vec[0]*Math.cos(angle) - vec[1]*Math.sin(angle),
        vec[1]*Math.cos(angle) + vec[0]*Math.sin(angle)
    ];
}

function to_unit(v){
    return numeric.div(v, numeric.norm2(v));
}

function get_bisector_vel(a,b){
    var angle_between = vector_angle(a,b);
    var n_angle = angle_between;
    if(n_angle < 0) n_angle += 2*Math.PI;

    var move_dir = to_unit(rotate(a, n_angle/2));
    var move_vel = 1/Math.cos((n_angle-Math.PI)/2);
    var move_vec = numeric.mul(move_dir, move_vel);
    return move_vec;
}

function adjacent_vert_intersection(a,b) {
    var t = intersect_edges(a.pos, numeric.add(a.pos, a.vel), b.pos, numeric.add(b.pos, b.vel)).a;
    if(t>0) return t;
    else return Infinity;
}

function scalar_project(v,axis){
  return numeric.dot(v, axis) / numeric.norm2(axis);
}

function vert_edge_intersection(p,e1,e2) {
    var edge_vec = numeric.sub(e2.pos, e1.pos);
    var edge_vel = to_unit(rotate(edge_vec, Math.PI/2));

    var intersect = intersect_edges(p.pos, numeric.add(p.pos, numeric.sub(p.vel, edge_vel)), e1.pos, e2.pos);
    var time = intersect.a;
    var position = intersect.b;

    var e1move = scalar_project(e1.vel, edge_vec)/numeric.norm2(edge_vec);
    var e2move = scalar_project(e2.vel, edge_vec)/numeric.norm2(edge_vec);

    if(time > 0 && time*e1move < position && position < 1+time*e2move){
        return time;
    } else {
        return Infinity;
    }
}

function deep_flatten(arr){
    var acc = [];
    for (var i = 0; i < arr.length; i++) {
        if(arr[i] instanceof Array){
            acc = acc.concat(deep_flatten(arr[i]));
        } else {
            acc.push(arr[i]);
        }
    }
    // console.log(acc);
    return acc;
}
function pair(arr){
    var acc = [];
    for (var i = 0; i < arr.length; i+=2) {
        acc.push([arr[i],arr[i+1]]);
    }
    return acc
}

var STRAIGHT_SKELETON_DEBUG = false;

function build_straight_skeleton(poly){
    var faces = [];
    var face_maps_int = [];
    var face_maps_ext = [];
    for (var i = 0; i < poly.length; i++) {
        var v1 = poly[i];
        var v2 = poly[(i+1) % poly.length];
        var face = {
            idx:i,
            source: i,
            direction: 'interior',
            vertices: [v1, v2, []],
            adjacent_faces: [i+poly.length, []],
        };
        faces.push(face);
        face_maps_int.push({
            face:face,
            vert_insertion: face.vertices[2],
            adj_insertion: face.adjacent_faces[1],
        });
    }
    for (var i = 0; i < poly.length; i++) {
        var v1 = poly[i];
        var v2 = poly[(i+1) % poly.length];
        var face = {
            idx: i+poly.length,
            source: i,
            direction: 'exterior',
            vertices: [v2, v1, []],
            adjacent_faces: [i, []],
        };
        face_maps_ext.unshift({
            face:face,
            vert_insertion: face.vertices[2],
            adj_insertion: face.adjacent_faces[1],
        });
        faces.push(face);
    }
    straight_skeleton_helper(poly, face_maps_int);
    var revpoly = poly.slice();
    revpoly.reverse();
    face_maps_ext.push(face_maps_ext.shift());
    straight_skeleton_helper(revpoly, face_maps_ext);


    for (var i = 0; i < faces.length; i++) {
        faces[i].vertices = pair(deep_flatten(faces[i].vertices));
        faces[i].adjacent_faces = deep_flatten(faces[i].adjacent_faces);
    }
    if(STRAIGHT_SKELETON_DEBUG){
        console.group("The full result:");
        for (var i = 0; i < faces.length; i++) {
            console.log(i, " with adjacencies ",faces[i].adjacent_faces);
        }
        console.groupEnd();
    }
    return faces;
}

function straight_skeleton_helper(subpoly, faces){
    // subpoly is a polygon
    // faces is a set of {face:face, insertion:index},
    // corresponding to each side of the polygon
    if(STRAIGHT_SKELETON_DEBUG){
        console.group("Running sshelper with ", subpoly.length, " vertices and faces:");
        for (var i = 0; i < faces.length; i++) {
            console.log(faces[i].face.idx, " with adjacencies ", JSON.stringify(faces[i].face.adjacent_faces));
        }
        console.groupEnd();
    }

    var vertices = [];
    for (var i = 0; i < subpoly.length; i++) {
        var v0 = subpoly[(i+subpoly.length-1) % subpoly.length];
        var v1 = subpoly[i];
        var v2 = subpoly[(i+1) % subpoly.length];

        var b = to_unit(numeric.sub(v0,v1));
        var a = to_unit(numeric.sub(v2,v1));

        var move_vec = get_bisector_vel(a,b);

        vertices.push({pos:v1, vel:move_vec});
    }

    var soonest_event_time = Infinity;
    var soonest_event = null;
    for (var i = 0; i < subpoly.length; i++) {
        var v1 = vertices[i];
        var v2 = vertices[(i+1) % subpoly.length];
        var time = adjacent_vert_intersection(v1,v2);
        if(time < soonest_event_time){
            soonest_event_time = time;
            soonest_event = {
                type: 'vert-vert',
                i1: i,
                i2: (i+1) % subpoly.length,
                v1: v1,
                v2: v2
            };
        }
    }
    for (var i = 0; i < subpoly.length; i++) {
        var v = vertices[i];
        for (var j = 0; j < subpoly.length; j++) {
            if((j-1+subpoly.length) % subpoly.length == i) continue;
            if(j==i) continue;
            if(((j+1) % subpoly.length) == i) continue;
            if(((j+2) % subpoly.length) == i) continue;
            var e1 = vertices[j];
            var e2 = vertices[(j+1) % subpoly.length];
            var time = vert_edge_intersection(v,e1,e2);
            if(time < soonest_event_time){
                soonest_event_time = time;
                soonest_event = {
                    type: 'vert-edge',
                    i: i,
                    ei1: j,
                    ei2: (j+1) % subpoly.length,
                    e1: e1,
                    e2: e2
                };
            }
        }
    }

    if(soonest_event_time == Infinity){
        if(STRAIGHT_SKELETON_DEBUG) console.log("Doing infinity case");
        // console.log("Inf");
        // Nothing intersected! Project to infinity vertices
        for (var i = 0; i < vertices.length; i++) {
            vertices[i].pos = move_pos(vertices[i].pos, vertices[i].vel, 100000);
        }
        for (var i = 0; i < vertices.length; i++) {
            var pfobj = faces[(i-1+vertices.length)%vertices.length];
            var cfobj = faces[i];
            var nfobj = faces[(i+1)%vertices.length];
            var cv = vertices[i];
            var nv = vertices[(i+1)%vertices.length];
            cfobj.vert_insertion.push(nv.pos, cv.pos);
            cfobj.adj_insertion.push(nfobj.face.idx,-1,pfobj.face.idx);
        }
    } else if(soonest_event.type == "vert-vert"){
        // Two vertices collided.
        // Shift all vertices by soonest_event_time forward.
        for (var i = 0; i < vertices.length; i++) {
            vertices[i].pos = move_pos(vertices[i].pos, vertices[i].vel, soonest_event_time);
        }
        // Connect the face corresponding to these vertices.
        var collision_pt = vertices[soonest_event.i1].pos;
        // console.log(vertices[soonest_event.i1].pos, vertices[soonest_event.i2].pos);
        lfobj = faces[(soonest_event.i1-1 + faces.length) % faces.length];
        mfobj = faces[soonest_event.i1];
        rfobj = faces[soonest_event.i2];

        if(faces.length == 3){
            if(STRAIGHT_SKELETON_DEBUG) console.log("Doing triangle contraction case");
            // All faces are contracting!
            lfobj.vert_insertion.push(collision_pt);
            mfobj.vert_insertion.push(collision_pt);
            rfobj.vert_insertion.push(collision_pt);

            lfobj.adj_insertion.push(mfobj.face.idx, rfobj.face.idx);
            mfobj.adj_insertion.push(rfobj.face.idx, lfobj.face.idx);
            rfobj.adj_insertion.push(lfobj.face.idx, mfobj.face.idx);
        } else{
            if(STRAIGHT_SKELETON_DEBUG) console.log("Doing merge case on ", soonest_event.i1, " and ", soonest_event.i2);
            // Remove one face
            var lv_rest = [];
            var rv_rest = [];
            lfobj.vert_insertion.push(collision_pt, lv_rest);
            mfobj.vert_insertion.push(collision_pt);
            rfobj.vert_insertion.push(rv_rest,collision_pt);

            var la_rest = [];
            var ra_rest = [];
            lfobj.adj_insertion.push(mfobj.face.idx,la_rest);
            mfobj.adj_insertion.push(rfobj.face.idx, lfobj.face.idx);
            rfobj.adj_insertion.push(ra_rest,mfobj.face.idx);

            var new_subpoly = [];
            var new_faces = [];

            var p_idx = (soonest_event.i1-1 + subpoly.length) % subpoly.length;
            var n_idx = (soonest_event.i2 + 1) % subpoly.length;
            new_subpoly.push(vertices[p_idx].pos);
            new_subpoly.push(collision_pt);

            new_faces.push({
                face:lfobj.face,
                vert_insertion: lv_rest,
                adj_insertion: la_rest
            });
            new_faces.push({
                face:rfobj.face,
                vert_insertion: rv_rest,
                adj_insertion: ra_rest
            });

            for (var i = n_idx; i != p_idx; i = ((i+1) % subpoly.length)) {
                new_subpoly.push(vertices[i].pos);
                new_faces.push(faces[i]);
            }

            if(STRAIGHT_SKELETON_DEBUG) console.group("Recursion:");
            straight_skeleton_helper(new_subpoly,new_faces);
            if(STRAIGHT_SKELETON_DEBUG) console.groupEnd();
        }
    } else if(soonest_event.type == "vert-edge"){
        // Vertex collided with an edge
        // Shift all vertices by soonest_event_time forward.
        if(STRAIGHT_SKELETON_DEBUG) console.log("Doing split case on ", soonest_event.i, " with ", soonest_event.ei1, " and ", soonest_event.ei2);
        for (var i = 0; i < vertices.length; i++) {
            vertices[i].pos = move_pos(vertices[i].pos, vertices[i].vel, soonest_event_time);
        }
        // Add the intersecting vertex to its corresponding faces
        var collision_pt = vertices[soonest_event.i].pos;
        var afobj = faces[(soonest_event.i-1 + faces.length) % faces.length];
        var bfobj = faces[soonest_event.i];
        var cfobj = faces[soonest_event.ei1];

        var av_rest = [];
        var bv_rest = [];
        var cv_a_rest = [];
        var cv_b_rest = [];
        afobj.vert_insertion.push(collision_pt, av_rest);
        bfobj.vert_insertion.push(bv_rest,collision_pt);
        cfobj.vert_insertion.push(cv_a_rest,collision_pt,cv_b_rest);

        var aa_rest = [];
        var ba_rest = [];
        var ca_a_rest = [];
        var ca_b_rest = [];
        afobj.adj_insertion.push(bfobj.face.idx,aa_rest);
        bfobj.adj_insertion.push(ba_rest,afobj.face.idx);
        cfobj.adj_insertion.push(ca_a_rest,ca_b_rest);

        // Split polygon into two pieces, and create corresponding face accessors
        var new_subpoly_a = [];
        var new_faces_a = [];
        var new_subpoly_b = [];
        var new_faces_b = [];

        var p_idx = (soonest_event.i-1 + subpoly.length) % subpoly.length;
        var n_idx = (soonest_event.i + 1) % subpoly.length;

        new_subpoly_a.push(vertices[p_idx].pos);
        new_subpoly_a.push(collision_pt);
        new_faces_a.push({
            face:afobj.face,
            vert_insertion: av_rest,
            adj_insertion: aa_rest
        });
        new_faces_a.push({
            face:cfobj.face,
            vert_insertion: cv_a_rest,
            adj_insertion: ca_a_rest
        });
        for (var i = soonest_event.ei2; i != p_idx; i = ((i+1) % subpoly.length)) {
            new_subpoly_a.push(vertices[i].pos);
            new_faces_a.push(faces[i]);
        }

        new_subpoly_b.push(vertices[soonest_event.ei1].pos);
        new_subpoly_b.push(collision_pt);
        new_faces_b.push({
            face:cfobj.face,
            vert_insertion: cv_b_rest,
            adj_insertion: ca_b_rest
        });
        new_faces_b.push({
            face:bfobj.face,
            vert_insertion: bv_rest,
            adj_insertion: ba_rest
        });
        for (var i = n_idx; i != soonest_event.ei1; i = ((i+1) % subpoly.length)) {
            new_subpoly_b.push(vertices[i].pos);
            new_faces_b.push(faces[i]);
        }

        // Recurse
        if(STRAIGHT_SKELETON_DEBUG) console.group("First recursion:");
        straight_skeleton_helper(new_subpoly_a,new_faces_a);
        if(STRAIGHT_SKELETON_DEBUG) console.groupEnd();
        if(STRAIGHT_SKELETON_DEBUG) console.group("Second recursion");
        straight_skeleton_helper(new_subpoly_b,new_faces_b);
        if(STRAIGHT_SKELETON_DEBUG) console.groupEnd();
    }
    if(STRAIGHT_SKELETON_DEBUG) {
        console.group("Finally:");
        for (var i = 0; i < faces.length; i++) {
            console.log(faces[i].face.idx, " with adjacencies ", JSON.stringify(faces[i].face.adjacent_faces));
        }
        console.groupEnd();
    }
}

function findPerpendiculars(poly, straightSkeleton) {
    // The straightSkeleton[i] should correspond to edge made up of poly[i] and poly[(i+1)%poly.length]
    // Do a depth first search to find perpendicular edges
    // Returns an array of 2 vertex pairs (straight skeleton vertex and vertex on polygon)

    var output = [];

    // Go through each polygon of the straight skeleton
    for (var p = 0; p < straightSkeleton.length; p++) {
        console.log("p=",p);

        // Go through vertices of the polygon
        var polygon = straightSkeleton[p];
        for (var v = 2; v < polygon.vertices.length; v++) {
            console.log("v=",v);

            var edge = findPerpEdge([polygon.vertices[0],polygon.vertices[1]], polygon.vertices[v]);

            // If the edge doesn't actually intersect with the cut polygon
            if(!verifyIntersect(polygon.vertices[0],polygon.vertices[1],edge[0])) {
                console.log("Not intersecting");
                console.log(polygon.vertices[0],polygon.vertices[1],edge[0]);
                continue;
            }
            output.push(edge);
            console.log("Perp added");
            console.log(edge);

            var vertex = edge[0];
            var pNew = polygon.adjacent_faces[0];
            var intersectEdge = [polygon.vertices[0],polygon.vertices[1]];

            // While not going to infinity, keep making angle bisectors
            while (pNew >= 0){
                polygon = straightSkeleton[pNew];
                var slope = findBisectorSlope(edge[1],edge[0],intersectEdge[0]);


                for (var i = 0; i < polygon.vertices.length; i++) {
                    var intersection = calculateIntersection(polygon.vertices[i],
                        polygon.vertices[(i+1)%polygon.vertices.length],vertex,slope);

                    if(verifyIntersect(polygon.vertices[i],
                        polygon.vertices[(i+1)%polygon.vertices.length],intersection)) {
                        output.push([intersection,vertex]);
                        vertex = intersection;
                        pNew = polygon.adjacent_faces[i];
                        intersectEdge = [polygon.vertices[i],polygon.vertices[(i+1)%polygon.vertices.length]];
                        break;
                    }
                }
            }
        }
    }
    return output;

    // var output = []

    // for (var i = 0; i < poly.length; i++) {
    //  var v0 = poly[i];
    //  var v1 = poly[(i+1)%poly.length];
    //  var face = straightSkeleton[i]


    //  // Get perpendicular slope
    //  var slope = -(v0[0]-v1[0]) / (v0[1]-v1[1]);


    //  // y-y1=m(x-x1)
    //  // y-(y1/m)+x1 = m
    //  // y+slope*v0[1]+v0[0] = x
    //  // y-face[j][1]/slope+face[j][0] = x
    //  for (var j = 0; j < face.length; j++) {
    //      if (face[j][0] !== v0[0] && face[j][1] !== v0[1] 
    //          &&face[j][0] !== v1[0] && face[j][1] !== v1[1]) {
    //          var x = (slope * v0[1]) + v0[0] - ((-1*face[j][1]/slope) + face[j][0]);
    //          var y = slope * (x - face[j][0]) + face[j][1];
    //          output.push([face[j],[x,y]]);
    //      }
    //  }
    // }

    // return output;
}

// function perpHelper(poly, straightSkeleton, perpEdges, changed) {
//  // Input polygon, straightSkeleton as array of polys, the perpendicular edges
//  // as array of arrays of edges, and an array of arrays of vertices

//  if (changed.length === 0) {
//      return straightSkeleton;
//  }



// }

function findInArray(array0, array1) {
    // Find index of array1 in array0
    // array0 is an array of 2 element arrays
    // array1 is a 2 element array
    // if array1 is not in array0, return -1
    for (var i = 0; i < array0.length; i++) {
        if (array0[i][0] === array1[0] && array0[i][1] === array1[1]) {
            return i;
        }
    }
    return -1;
}


// DEPRECATED
function flattenSkeleton(straightSkeleton) {
    // Flattens vertices down to just coordinates again

    var output = [];
    for (var i = 0; i < straightSkeleton.length; i++) {
        var polygon = [[],straightSkeleton[i][1]];
        for (var j = 0; j < straightSkeleton[i][0].length; j++) {
            polygon[0].push(straightSkeleton[i][0][j][0]);
        }

        output.push(polygon);
    }
    return output;
}

function findPerpEdge(edge, vertex) {
    // Input an edge as an array of vertices and a vertex


    // Slope of perpendicular line
    var slope = -(edge[0][0]-edge[1][0]) / (edge[0][1]-edge[1][1]);
    console.log("perp slope = ", slope)
    return [calculateIntersection(edge[0],edge[1],vertex,slope), vertex]

}

function calculateIntersection(v0, v1, v2, slope) {
    // Find intersection of line defined by v0 and v1 and line defined by v2 and slope

    var slopeEdge = (v0[1]-v1[1]) / (v0[0]-v1[0]);
    var x = (slopeEdge*v0[0]-v0[1]-slope*v2[0]+v2[1]) / (slopeEdge - slope);
    var y = slope * (x - v2[0]) + v2[1];
    return [x,y];
}

function verifyIntersect(v0,v1,vtest) {
    // See if vtest is between v0 and v1 assuming all are colinear
    var test0 = Math.sqrt(Math.pow(vtest[0]-v0[0],2)+Math.pow(vtest[1]-v0[1],2));
    var test1 = Math.sqrt(Math.pow(vtest[0]-v1[0],2)+Math.pow(vtest[1]-v1[1],2));
    var dist = Math.sqrt(Math.pow(v1[0]-v0[0],2)+Math.pow(v1[1]-v0[1],2));
    return (dist > test0 && dist > test1);
}

function findBisectorSlope(v0, v1, v2) {
    // Find slope such that edge v1,v2 is the bisector of the angle
    // between the line formed by the slope and v1 and v0,v1

    var a = [v0[0]-v1[0], v0[1]-v1[1]];
    var b = [v2[0]-v1[0], v2[1]-v1[1]];
    var angle = vector_angle(a,b);
    var vec = rotate(a,angle);
    return vec[1]/vec[0];

}