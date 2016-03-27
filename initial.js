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

function motorcycleGraph(poly){
	// Set up the graph structure
	var graph_verts = [];
	var graph_edges = [];

	// Compute the (interior/exterior) motorcycle graph for a (ccw/cw) polygon
	var walls = [];
	for (var i = 0; i < poly.length; i++) {
		// add graph edge
		graph_edges.push([i, (i+1) % poly.length]);

		var v1 = poly[i];
		var v2 = poly[(i+1) % poly.length];
		walls.push({
			a:v_1,
			b:v_2,
			segments:[{
				t_start: 0,
				p_end: 1,
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

		var a = to_unit(numeric.sub(v1,v0));
		var b = to_unit(numeric.sub(v1,v2));

		var angle_between = vector_angle(a,b);

		var move_dir = to_unit(rotate(a, angle_between/2));
		var move_vel = 1/Math.cos((angle_between-Math.pi)/2);
		var move_vec = numeric.mul(motor_dir, motor_vel);

		if(angle_between >= Math.PI){
			// Convex vertex
			graph_verts.push({
				type:'convex',
				pos: v1.slice(),
				vel: move_vec,
			});
		} else if (angle_between <= Math.PI) {
			// Reflex vertex
			graph_verts.push({
				type:'reflex',
				pos: v1.slice(),
				vel: move_vec,
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
			return null;
		}
	}

	// Find all motorcycle collisions
	collisions = [];
	for (var i = 0; i < motorcycles.length; i++) {
		var m = motorcycles[i];
		for (var j = 0; j < walls.length; j++) {
			var intersect = intersect_edges(m.pos, numeric.add(m.pos, m.vel), wall[j].a, wall[j].b);
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
				} else if(intersect.b > intersect.a) {
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
				return null;
			}
		} else {
			// The crashing motorcycle hit another motorcycle trace
			var crashee = motrocycles[evt.crashee];
			if(crashee.dies <= time){
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
					return null;
				}
			}
		}
	}

	// At this point, we have created the full motorcycle graph
	var clean_edges = graph_edges.filter(function(x){return x !== null;});
	return [graph_verts, clean_edges];
}

function getCollapseTime(edge) {
	
	return 1;
}

function findStraightSkeleton(poly) {
	var wStar = motorcycle_graph(poly);
	var vertices = wStar[0];
	var edges = wStar[1];
	var ss = []; // The skeleton is represented as a list of polygons

	// Put edges of wStar in a priority queue
	var pq = new PriorityQueue({comparator: getCollapseTime});

	for (var i = 0; i < edges.length; i++) {
		pq.queue(edges[i]);
	}

	// Pop an edge, handle its event type, and update the queue
	while (pq.length != 0) {
		var edgeToHandle = pq.dequeue();

		switch(edgeToHandle.type) {
			case "edge":
				// Add convex SS arcs

				// Merge vertices
				// Special case: Check whether triangle collapse
				break;
			case "split":
				// Add reflex SS arc
				// If left side of edge collapsed, add arcs.
				// Otherwise, add new convex vertices
				break;
			case "start":
				// Change vertex type to moving steiner
				// Split edge
				break;
			case "switch":
				// If v (what is v?) was reflex, make it moving steiner
				break;
			case default:
				// Remove the edge
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
	// Returns an array of 2 vertex pairs (straight skeleton vertex and vertex on polygon)

	var output = []

	for (var i = 0; i < poly.length; i++) {
		var v0 = poly[i];
		var v1 = poly[(i+1)%poly.length];
		var face = straightSkeleton[i]


		// Get perpendicular slope
		var slope = -(v0[0]-v1[0]) / (v0[1]-v1[1]);


		// y-y1=m(x-x1)
		// y-(y1/m)+x1 = m
		// y+slope*v0[1]+v0[0] = x
		// y-face[j][1]/slope+face[j][0] = x
		for (var j = 0; j < face.length; j++) {
			if (face[j][0] !== v0[0] && face[j][1] !== v0[1] 
				&&face[j][0] !== v1[0] && face[j][1] !== v1[1]) {
				var x = (slope * v0[1]) + v0[0] - ((-1*face[j][1]/slope) + face[j][0]);
				var y = slope * (x - face[j][0]) + face[j][1];
				output.push([face[j],[x,y]]);
			}
		}
	}

	return output;
}
