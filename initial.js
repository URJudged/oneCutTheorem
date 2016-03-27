
function vector_angle(x1,y1,x2,y2){
  var dot = x1*x2 + y1*y2;
  var cross_z = x1*y2 - y1*x2;
  var angle = Math.atan2(cross_z,dot);
  return angle;
}

function findStraightSkeleton(poly){
	// Find the polygon sections that make up the straight skeleton
	// Output a list of lists of vertices

	// Get angle bisectors
	var output = [];
	var angles = [];
	var lengths = [];
	for (var i = 0; i < poly.length; i++) {
		var v0 = poly[(i+poly.length-1) % poly.length];
		var v1 = poly[(i+1) % poly.length];

		// Get angle bisector and edge lengths denoted by first vertex. Start storing faces
		angles.push(vector_angle(v0[0], v0[1], v1[0], v1[1]) * 0.5);
		lengths.push(Math.sqrt(Math.pow((v0[0]+v1[0]),2)+Math.pow((v0[1]+v1[1]),2)))
		output.push([v0, v1]);
	}


	// NOT SURE IF THIS IS CORRECT
	// Get indices from shortest to longest edge denoted by first vertex
	var toSort = [];
	for (var i = 0; i < lengths.length; i++) {
		toSort.push([lengths[i],i]);
	}
	toSort.sort(function(left, right) {
    	return left[0] < right[0] ? -1 : 1;
  		});
	var lengthOrder = [];
	for (var i = 0; i < toSort.length; i++) {
		lengthOrder.push(toSort[i][0]);
	}

	for (var i = 0; i < lengthOrder.length; i++) {

	}

}

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