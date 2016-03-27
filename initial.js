
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

function motorcycleGraph(poly, reversed, orig_verts, orig_edges){
	// Compute the motorcycle graph
	if(reversed){
		poly = poly.slice();
		poly.reverse();
	}

	// Set up the graph structure
	var graph_verts = orig_verts || [];
	var graph_edges = orig_edges || [];

	var walls = [];
	for (var i = 0; i < poly.length; i++) {
		// add graph edge
		graph_edges.push([i, (i+1) % poly.length]);

		var v1 = poly[i];
		var v2 = poly[(i+1) % poly.length];
		walls.push({
			a:v1,
			b:v2,
			segments:[{
				t_start: 0,
				t_end: 1,
				edge: i
			}],
		});
	}

	// Generate initial graph verts and motorcycles
	var motorcycles = [];
	for (var i = 0; i < poly.length; i++) {
		var v0 = poly[(i+poly.length-1) % poly.length];
		var v1 = poly[i];
		var v2 = poly[(i+1) % poly.length];

		var b = to_unit(numeric.sub(v0,v1));
		var a = to_unit(numeric.sub(v2,v1));

		var angle_between = vector_angle(a,b);
		var n_angle = angle_between;
		if(n_angle < 0) n_angle += 2*Math.PI;

		var move_dir = to_unit(rotate(a, n_angle/2));
		var move_vel = 1/Math.cos((n_angle-Math.PI)/2);
		var move_vec = numeric.mul(move_dir, move_vel);

		// console.log({ang:angle_between, a:a,b:b, dir:move_dir, vel:move_vel});

		if(angle_between > 0){
			// Convex vertex
			graph_verts.push({
				type:'convex',
				pos: v1.slice(),
				vel: move_vec,
				adjacent: [i, (i-1+poly.length)%poly.length],
			});
		} else if (angle_between < 0) {
			// Reflex vertex
			graph_verts.push({
				type:'reflex',
				pos: v1.slice(),
				vel: move_vec,
				adjacent: [i, (i-1+poly.length)%poly.length],
			});
			motorcycles.push({
				pos:v1.slice(),
				vel:move_vec,
				dies:Infinity,
				segments:[],
				last_time:0,
				last_vert:i,
			});
		} else{
			// abandon all hope
			console.error("abandon all hope");
			return null;
		}
	}

	// Find all motorcycle collisions
	collisions = [];
	for (var i = 0; i < motorcycles.length; i++) {
		var m = motorcycles[i];
		for (var j = 0; j < walls.length; j++) {
			var intersect = intersect_edges(m.pos, numeric.add(m.pos, m.vel), walls[j].a, walls[j].b);
			if(intersect.a > 0 && intersect.b >= 0 && intersect.b <= 1 ){
				// This motorcycle hit a wall
				collisions.push({
					t:intersect.a,
					pos:move_pos(m.pos, m.vel, intersect.a),
					crasher:i,
					hit_wall: true,
					crashee:j,
					crashee_time: intersect.b,
				});
			}
		}
		for (var j = 0; j < i; j++) {
			var m2 = motorcycles[j];
			var intersect = intersect_edges(m.pos, numeric.add(m.pos, m.vel), m2.pos, numeric.add(m2.pos, m2.vel));
			if(intersect.a > 0 && intersect.b > 0){
				// Two motorcycles hit each other
				if(intersect.a < intersect.b){
					// m2 is the crasher
					collisions.push({
						t:intersect.b,
						pos:move_pos(m2.pos, m2.vel, intersect.b),
						crasher:j,
						hit_wall: false,
						crashee:i,
						crashee_time: intersect.a,
					});
				} else if(intersect.b < intersect.a) {
					// m1 is the crasher
					collisions.push({
						t:intersect.a,
						pos:move_pos(m.pos, m.vel, intersect.a),
						crasher:i,
						hit_wall: false,
						crashee:j,
						crashee_time: intersect.b,
					});
				}  else{
					// abandon all hope
					console.error("abandon all hope");
					return null;
				}
			}
		}
	}

	// Sort the collisions by time
	function sort_collisions_fn(a,b){
		return a.t - b.t;
	}
	collisions.sort(sort_collisions_fn);

	// Process the graph
	function help_insert_segment(evt,new_vert_idx,crashee){
		for (var i = 0; i < crashee.segments.length; i++) {
			var seg = crashee.segments[i];
			if(seg.t_start < evt.crashee_time && evt.crashee_time < seg.t_end){
				// Found the right segment to insert into
				// Add new edges
				graph_edges.push([graph_edges[seg.edge][0], new_vert_idx]);
				graph_edges.push([new_vert_idx, graph_edges[seg.edge][1]]);
				// Reconfigure adjacent
				var adj1 = graph_verts[graph_edges[seg.edge][0]].adjacent;
				adj1.splice(adj1.indexOf(seg.edge),1,graph_edges.length-1);
				var adj2 = graph_verts[graph_edges[seg.edge][1]].adjacent;
				adj2.splice(adj2.indexOf(seg.edge),1,graph_edges.length-2);
				graph_verts[new_vert_idx].adjacent.push(graph_edges.length-1,graph_edges.length-2);
				// Remove the old edge
				graph_edges[seg.edge] = null;
				// Create new segments
				var s1 = {
					t_start: seg.t_start,
					t_end: evt.crashee_time,
					edge: graph_edges.length-2
				};
				var s2 = {
					t_start: evt.crashee_time,
					t_end: seg.t_end,
					edge: graph_edges.length-1
				};
				// Remove old segments
				crashee.segments.splice(i,1,s1,s2);
				break;
			}
		}
		if(i == crashee.segments.length){
			// We didn't find a segment that works
			return false;
		}
		return true;
	}

	for (var i = 0; i < collisions.length; i++) {
		var evt = collisions[i];
		var time = evt.t;
		var crasher = motorcycles[evt.crasher];
		if(crasher.dies != Infinity){
			// The crashing motorcycle is already dead.
			continue;
		}
		if(evt.hit_wall){
			// The crashing motorcycle hit a wall.
			var crashee = walls[evt.crashee];

			// Create a wall-crash vertex
			var new_vert_idx = graph_verts.length;
			var wall_vec = numeric.sub(crashee.b, crashee.a);
			var new_speed = 1/Math.abs(Math.sin(vector_angle(wall_vec, crasher.vel)));
			var new_vel = numeric.mul(to_unit(crasher.vel), -new_speed);
			var vertex = {
				type:'moving_steiner',
				pos: evt.pos,
				vel: new_vel,
				adjacent: [graph_edges.length],
			};

			// Add it appropriately
			graph_verts.push(vertex);

			graph_edges.push([crasher.last_vert, new_vert_idx]);
			crasher.segments.push({
				t_start: crasher.last_time,
				t_end: time,
				edge: graph_edges.length-1
			});
			crasher.dies = time;
			crasher.last_time = time;
			crasher.last_vert = new_vert_idx;

			if(!help_insert_segment(evt,new_vert_idx,crashee)){
				// abandon all hope
				console.error("abandon all hope");
				return null;
			}
		} else {
			// The crashing motorcycle hit another motorcycle trace
			var crashee = motorcycles[evt.crashee];
			if(crashee.dies <= evt.crashee_time){
				// The crashee motorcycle died before getting here, so
				// we are good to go
				continue;
			}

			// Create a motorcycle-crash vertex
			var new_vert_idx = graph_verts.length;
			var vertex = {
				type:'resting_steiner',
				pos: evt.pos,
				vel: [0,0],
				adjacent: [graph_edges.length],
			};

			// Add it appropriately
			graph_verts.push(vertex);

			graph_edges.push([crasher.last_vert, new_vert_idx]);
			crasher.segments.push({
				t_start: crasher.last_time,
				t_end: time,
				edge: graph_edges.length-1
			});
			crasher.dies = time;
			crasher.last_time = time;
			crasher.last_vert = new_vert_idx;

			if(evt.crashee_time > crashee.last_time){
				// We hit the 'active' segment of the crashee
				// Add a new fixed segment
				vertex.adjacent.push(graph_edges.length);
				graph_verts[crashee.last_vert].adjacent.push(graph_edges.length);
				graph_edges.push([crashee.last_vert, new_vert_idx]);

				crashee.segments.push({
					t_start: crashee.last_time,
					t_end: time,
					edge: graph_edges.length-1
				});
				crashee.last_time = time;
				crashee.last_vert = new_vert_idx;
			} else {
				// We hit an existing fixed segment
				if(!help_insert_segment(evt,new_vert_idx,crashee)){
					// abandon all hope
					console.error("abandon all hope");
					return null;
				}
			}
		}
	}

	// Clean up any infinity motorcycles
	var infinities = [];
	for (var i = 0; i < motorcycles.length; i++) {
		var m = motorcycles[i];
		if(m.dies == Infinity) {
			// This motorcycle is still alive!
			// Create an infinity vertex for it
			var new_vert_idx = graph_verts.length;
			var vertex = {
				type:'infinity',
				pos: move_pos(m.pos, m.vel, 10000000),
				vel: m.vel,
				adjacent: [graph_edges.length],
			};
			// Add it appropriately
			graph_verts.push(vertex);
			graph_edges.push([m.last_vert, new_vert_idx]);

			infinities.push(new_vert_idx);
		}
	}
	for (var i = 0; i < infinities.length; i++) {
		// Connect all infinities together
		var first = infinities[i];
		var second = infinities[(i+1) % infinities.length];
		graph_verts[first].adjacent.push(graph_edges.length);
		graph_verts[second].adjacent.push(graph_edges.length);
		graph_edges.push([first, second]);
	}

	// At this point, we have created the full motorcycle graph
	// var clean_edges = graph_edges.filter(function(x){return x !== null;});
	return [graph_verts, clean_edges];
}

function doubleMotorcycleGraph(poly){
	var wStarVerts = [];
	var wStarEdges = [];
	motorcycle_graph(poly, false, wStarVerts, wStarEdges);
	var reversed_start_v = wStarVerts.length;
	var reversed_start_e = wStarEdges.length;
	motorcycle_graph(poly, true, wStarVerts, wStarEdges);
	return [wStarVerts,wStarEdges];
}

function getCollapseTime(edge) {
	
	return 1;
}

function findStraightSkeleton(poly) {
	var wStarVerts = [];
	var wStarEdges = [];
	motorcycle_graph(poly, false, wStarVerts, wStarEdges);
	var reversed_start_v = wStarVerts.length;
	var reversed_start_e = wStarEdges.length;
	motorcycle_graph(poly, true, wStarVerts, wStarEdges);

	var ss = []; // The skeleton is represented as a list of polygons
	for (var i = 0; i < poly.length; i++) {
		var v1 = poly[i];
		var v2 = poly[(i+1) % poly.length];
		var face = {
			source: i,
			direction: 'interior',
			vertices: [v1, v2],
			adjacent_faces: [i+poly.length],
		};
		var wSv1 = wStarVerts[i];
		var wSv2 = wStarVerts[(i+1) % poly.length];

		wSv1.face_front = face;
		wSv2.face_back = face;

		ss.push(face);
	}
	for (var i = 0; i < poly.length; i++) {
		var v1 = poly[i];
		var v2 = poly[(i+1) % poly.length];
		var face = {
			source: i,
			direction: 'exterior',
			vertices: [v2, v1],
			adjacent_faces: [i-poly.length],
		};
		var wSv2 = wStarVerts[reversed_start_v + (i+1) % poly.length];
		var wSv1 = wStarVerts[reversed_start_v + i];

		wSv2.face_front = face;
		wSv1.face_back = face;

		ss.push(face);
	}

	var edgeTime = [];

	function getCollapseTime(edge) {
		var v0 = wStarVerts[edge[0]];
		var v1 = wStarVerts[edge[1]];
		var vel0 = v0.vel;
		var vel1 = v1.vel;
		
		if (eq(vel0,[0,0])) {
			return numeric.norm2(numeric.sub(v1,v0))/numeric.norm2(vel1);
		} else if(eq(vel1,[0,0])) {
			return numeric.norm2(numeric.sub(v1,v0))/numeric.norm2(vel0);
		} else {
			return intersect_edges(v0, numeric.add(v0, vel0), v1, numeric.add(v1, vel1)).a;
		}
	}

	function getTime(obj){
		return obj.time;
	}

	// Put edges of wStar in a priority queue
	var pq = new PriorityQueue({comparator: getCollapseTime});

	for (var i = 0; i < edges.length; i++) {
		if(edges[i] == null) continue;
		pq.queue({
			idx:i,
			time:getCollapseTime(wStarEdges[i]),
			edge:wStarEdges[i],
		});
	}

	// Pop an edge, handle its event type, and update the queue
	while (pq.length != 0) {
		var edgeToHandle = pq.dequeue();
		if(wStarEdges[edgeToHandle.i] == null) continue;
		
		var v0 = wStar[edgeToHandle.edge[0]];
		var v1 = wStar[edgeToHandle.edge[1]];

		if(v0.type > v1.type){
			var tmp = v0;
			v0 = v1;
			v1 = tmp;
		}

		// Edge event
		if (v0.type == "convex" && v1.type == "convex") {
			// Create new vertex
			var v_new = {
				type:'convex',
				pos: move_pos(v0.pos, v0.vel, ,
				vel: move_vec,
				adjacent: [i, (i-1+poly.length)%poly.length],
			}
			// Connect faces appropriately
			
		}
		// Split events
		else if (v0.type == "moving_steiner" && v1.type == "reflex") {

		}
		// Start events
		else if (v0.type == "reflex" && v1.type == "resting_steiner") {

		}
		else if (v0.type == "moving_steiner" && v1.type == "resting_steiner") {

		}
		// Switch events
		else if (v0.type == "convex" && v1.type == "moving_steiner") {

		}
		else if (v0.type == "convex" && v1.type == "reflex") {

		}
		// Else, two moving steiners
		else {

		}
	}

	return ss;
}
	// // Find the polygon sections that make up the straight skeleton
	// // Output a list of lists of vertices

	// // Get angle bisectors
	// var output = [];
	// var angles = [];
	// var lengths = [];
	// for (var i = 0; i < poly.length; i++) {
	// 	var v0 = poly[(i+poly.length-1) % poly.length];
	// 	var v1 = poly[(i+1) % poly.length];

	// 	// Get angle bisector and edge lengths denoted by first vertex. Start storing faces
	// 	angles.push(vector_angle(v0[0], v0[1], v1[0], v1[1]) * 0.5);
	// 	lengths.push(Math.sqrt(Math.pow((v0[0]+v1[0]),2)+Math.pow((v0[1]+v1[1]),2)))
	// 	output.push([v0, v1]);
	// }


	// // NOT SURE IF THIS IS CORRECT
	// // Get indices from shortest to longest edge denoted by first vertex
	// var toSort = [];
	// for (var i = 0; i < lengths.length; i++) {
	// 	toSort.push([lengths[i],i]);
	// }
	// toSort.sort(function(left, right) {
 //    	return left[0] < right[0] ? -1 : 1;
 //  		});
	// var lengthOrder = [];
	// for (var i = 0; i < toSort.length; i++) {
	// 	lengthOrder.push(toSort[i][0]);
	// }

	// for (var i = 0; i < lengthOrder.length; i++) {
		
	// }
// }

function findPerpendiculars(poly, straightSkeleton) {
	// The straightSkeleton[i] should correspond to edge made up of poly[i] and poly[(i+1)%poly.length]
	// Do a depth first search to find perpendicular edges
	// Returns an array of 2 vertex pairs (straight skeleton vertex and vertex on polygon)

	var output = [];

	// Go through each polygon of the straight skeleton
	for (var p = 0; p < straightSkeleton.length; p++) {

		// Go through vertices of the polygon
		var polygon = straightSkeleton[p];
		for (var v = 2; v < polygon.vertices.length; v++) {

			var edge = findPerpEdge([polygon.vertices[0],polygon.vertices[1]], polygon.vertices[v]);

			// If the edge doesn't actually intersect with the cut polygon
			if(!verifyIntersect(polygon.vertices[0],polygon.vertices[1],edge[0])) {
				continue;
			}
			output.push(edge);

			var vertex = edge[0];
			var pNew = polygon.adjacent_faces[0];
			var intersectEdge = [polygon.vertices[0],polygon.vertices[1]];

			// While not going to infinity, keep making angle bisectors
			while (pNew >= 0){
				polygon = straightSkeleton[pNew];
				var slope = findBisectorSlope(edges[1],edges[0],intersectEdge[0]);


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
	// 	var v0 = poly[i];
	// 	var v1 = poly[(i+1)%poly.length];
	// 	var face = straightSkeleton[i]


	// 	// Get perpendicular slope
	// 	var slope = -(v0[0]-v1[0]) / (v0[1]-v1[1]);


	// 	// y-y1=m(x-x1)
	// 	// y-(y1/m)+x1 = m
	// 	// y+slope*v0[1]+v0[0] = x
	// 	// y-face[j][1]/slope+face[j][0] = x
	// 	for (var j = 0; j < face.length; j++) {
	// 		if (face[j][0] !== v0[0] && face[j][1] !== v0[1] 
	// 			&&face[j][0] !== v1[0] && face[j][1] !== v1[1]) {
	// 			var x = (slope * v0[1]) + v0[0] - ((-1*face[j][1]/slope) + face[j][0]);
	// 			var y = slope * (x - face[j][0]) + face[j][1];
	// 			output.push([face[j],[x,y]]);
	// 		}
	// 	}
	// }

	// return output;
}

// function perpHelper(poly, straightSkeleton, perpEdges, changed) {
// 	// Input polygon, straightSkeleton as array of polys, the perpendicular edges
// 	// as array of arrays of edges, and an array of arrays of vertices

// 	if (changed.length === 0) {
// 		return straightSkeleton;
// 	}



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

	// y-y1 = m(x-x1)
	// -y1/m + x1 = x

	// Slope of perpendicular line
	var slope = -(edge[0][0]-edge[1][0]) / (edge[0][1]-edge[1][1]);
	return [calculateIntersection(edge[0],edge[1],vertex,slope), vertex]

}

function calculateIntersection(v0, v1, v2, slope) {
	// Find intersection of line defined by v0 and v1 and line defined by v2 and slope

	var slopeEdge = (v0[1]-v1[1]) / (v0[0]-v1[0]);
	var x = (-1*v0[1]/slopeEdge) + v0[0] - ((-1*v2[1]/slope) + v2[0]);
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
