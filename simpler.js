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
    return intersect_edges(a.pos, numeric.add(a.pos, a.vel), b.pos, numeric.add(b.pos, b.vel)).a;
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

function straight_skeleton(poly){
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
            vertices: [v1, v2],
            adjacent_faces: [i+poly.length],
        };
        faces.push(face);
        face_maps_int.push({
            face:face,
            insertion: 2
        });
    }
    for (var i = 0; i < poly.length; i++) {
        var v1 = poly[i];
        var v2 = poly[(i+1) % poly.length];
        var face = {
            idx: i+poly.length,
            source: i,
            direction: 'exterior',
            vertices: [v2, v1],
            adjacent_faces: [i-poly.length],
        };
        face_maps_ext.unshift({
            face:face,
            insertion: 2
        });
        faces.push(face);
    }
    straight_skeleton_helper(poly, face_maps_int);
    var revpoly = poly.slice();
    revpoly.reverse();
    straight_skeleton_helper(revpoly, face_maps_ext);

    return faces;
}

function straight_skeleton_helper(subpoly, faces){
    // subpoly is a polygon
    // faces is a set of {face:face, insertion:index},
    // corresponding to each side of the polygon
    var vertices = [];
    for (var i = 0; i < subpoly.length; i++) {
        var v0 = poly[(i+poly.length-1) % poly.length];
        var v1 = poly[i];
        var v2 = poly[(i+1) % poly.length];

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
            if(j==i || ((j+1) % subpoly.length) == i)
                continue;
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
        // Nothing intersected! Project to infinity vertices
        // TODO TODO
    } else if(soonest_event.type == "vert-vert"){
        // Two vertices collided.
        // Shift all vertices by soonest_event_time forward.
        // Connect the face corresponding to these vertices.
        // Replace the two vertices with one in the polygon
        // Recurse
    } else if(soonest_event.type == "vert-edge"){
        // Vertex collided with an edge
        // Shift all vertices by soonest_event_time forward.
        // Add the intersecting vertex to its corresponding faces
        // Split polygon into two pieces, and create corresponding face accessors
        // Recurse
    }
}

