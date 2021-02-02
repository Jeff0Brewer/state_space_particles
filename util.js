//multiply vector by scalar
function mult_scalar(vec, s){ 
	let o = vec.slice();
	for(let i = 0; i < o.length; i++)
		o[i] = o[i]*s;
	return o;
}

//find magnitude of vector
function mag(vec){ 
	let s = 0;
	for(let i = 0; i < vec.length; i++)
		s += Math.pow(vec[i], 2);
	return Math.sqrt(s);
}

//find normal of vector
function norm(vec){ 
	return mult_scalar(vec, 1/mag(vec));
}

//find sum of two vectors
function add(a, b){
	let o = [];
	for(let i = 0; i < a.length; i++)
		o.push(a[i]+b[i]);
	return o;
}

//find diff of two vectors
function sub(a, b){
	let o = [];
	for(let i = 0; i < a.length; i++)
		o.push(a[i]-b[i]);
	return o;
}

//find dot product of two vectors
function dot(a, b){
	let o = 0;
	for(let i = 0; i < a.length; i++)
		o += a[i]*b[i];
	return o;
}

//find cross product of 3d vectors
function cross3(a, b){
	return [a[1]*b[2] - a[2]*b[1],
			a[2]*b[0] - a[0]*b[2],
			a[0]*b[1] - a[1]*b[0]];
}

// map v from bounds a to bounds b
function map(v, a, b){
	return (v - a[0])/(a[1] - a[0])*(b[1] - b[0]) + b[0];
}

//find distance between point and plane
function dist_point_plane(pt, cnst){
	return (cnst[0]*pt[0] + cnst[1]*pt[1] + cnst[2]*pt[2] + cnst[3])/Math.sqrt(cnst[0]*cnst[0] + cnst[1]*cnst[1] + cnst[2]*cnst[2]);
}