IND = { // px, py, pz, vx, vy, vz, fx, fy, fz, m
	POS: 0, //position
	VEL: 3, //velocity
	FOR: 6, //force
	MAS: 9, //mass
	FPP: 10 //floats per particle
}
APRX_0 = [0, 0, .00000000000001];

class PartSys{
	constructor(num){
		this.num = num;
		this.time_factor = 1.0;

		this.s1 = new Float32Array(IND.FPP*num);
		this.s1dot = new Float32Array(IND.FPP*num);
		this.s2 = new Float32Array(IND.FPP*num);

		this.F = [];
		this.C = [];
	}

	init(center, size, vel, m_b, F, C){
		this.F = F;
		this.C = C;

		let p_b = [[-size, size], [-size, size], [-size, size]];
		for(let i = 0; i < this.s1.length; i++){
			let v_ind = i % IND.FPP;
			if(v_ind < IND.POS + 3)
				this.s1[i] = map(Math.random(), [0, 1], p_b[v_ind]) + center[v_ind];
			else if (v_ind < IND.VEL + 3)
				this.s1[i] = (Math.random() > .5 ? -1 : 1)*vel*Math.random();
			else if (v_ind < IND.FOR + 3)
				this.s1[i] = 0;
			else if (v_ind == IND.MAS)
				this.s1[i] = map(Math.random(), [0, 1], m_b);

			this.s1dot[i] = 0;
			this.s2[i] = this.s1[i];
		}

		let FC_data_len = 0;
		for(let i = 0; i < this.F.length; i++){
			FC_data_len += this.F[i].data_len;
		}
		for(let i = 0; i < this.C.length; i++){
			FC_data_len += this.C[i].data_len;
		}
		this.FC_data_num = FC_data_len/3;
	}

	applyAllForces(s, F){
		for(let n = 0; n < this.num; n++){
			for(let i = 0; i < 3; i++){
				s[n*IND.FPP + IND.FOR + i] = 0;
			}
		}
		for(let i = 0; i < F.length; i++){
			F[i].apply_force(s);
		}
	}

	dotFinder(s){
		let sdot = new Float32Array(IND.FPP*this.num);
		for(let n = 0; n < this.num; n++){
			let acc = mult_scalar(s.slice(n*IND.FPP + IND.FOR, n*IND.FPP + IND.FOR + 3), 1/s[n*IND.FPP + IND.MAS]);
			for(let i = 0; i < acc.length; i++){
				sdot[n*IND.FPP + IND.VEL + i] = acc[i];
				sdot[n*IND.FPP + IND.POS + i] = s[n*IND.FPP + IND.VEL + i];
			}
		}
		return sdot;
	}

	solver(elapsed){
		for(let i = 0; i < this.s1.length; i++){
			this.s2[i] = this.s1[i] + this.s1dot[i]*elapsed/1000*this.time_factor;
		}
	}

	doConstraint(s1, s2, C){
		for(let i = 0; i < C.length; i++){
			C[i].constrain(s1, s2);
		}
	}

	render(gl, gl_buf){
		let pos_buf = new Float32Array(this.num*3 + this.FC_data_num*3);
		let pos_ind = 0;
		for(let n = 0; n < this.num; n++){
			for(let i = 0; i < 3; i++, pos_ind++){
				pos_buf[pos_ind] = this.s2[n*IND.FPP + IND.POS + i];
			}
		}
		for(let i = 0; i < this.F.length; i++){
			let data = this.F[i].data_len > 0 ? this.F[i].get_buf_data(this.s2) : [];
			for(let j = 0; j < data.length; j++, pos_ind++){
				pos_buf[pos_ind] = data[j];
			}
		}
		for(let i = 0; i < this.C.length; i++){
			let data = this.C[i].data_len > 0 ? this.C[i].get_buf_data(this.s2) : [];
			for(let j = 0; j < data.length; j++, pos_ind++){
				pos_buf[pos_ind] = data[j];
			}
		}
		gl.bindBuffer(gl.ARRAY_BUFFER, gl_buf);
		gl.bufferSubData(gl.ARRAY_BUFFER, 0, pos_buf);
	}

	swap(){
		this.s1.set(this.s2);
	}
}

class SpringForcer{
	constructor(length, strength, damping, ind_a, ind_b){
		this.len = length;
		this.str = strength;
		this.dmp = damping;
		this.inds = [ind_a, ind_b];

		this.data_len = 6;
	}

	apply_force(s){
		let pos_diff = sub(s.slice(this.inds[0]*IND.FPP + IND.POS, this.inds[0]*IND.FPP + IND.POS + 3), s.slice(this.inds[1]*IND.FPP + IND.POS, this.inds[1]*IND.FPP + IND.POS + 3));
		let vel_diff = sub(s.slice(this.inds[0]*IND.FPP + IND.VEL, this.inds[0]*IND.FPP + IND.VEL + 3), s.slice(this.inds[1]*IND.FPP + IND.VEL, this.inds[1]*IND.FPP + IND.VEL + 3));
		if(mag(pos_diff) == 0)
			pos_diff = APRX_0;
		if(mag(vel_diff) == 0)
			vel_diff = APRX_0;
		let d = mag(pos_diff) - this.len;
		let dir = norm(pos_diff);
		vel_diff = mult_scalar(dir, dot(vel_diff, dir));

		let f = mult_scalar(dir, this.str*Math.pow(d, 2)*Math.sign(d) - (dot(vel_diff, dir) > 0 ? -1 : 1)*mag(vel_diff)*this.dmp);
		for(let i = 0; i < this.inds.length; i++){
			for(let j = 0; j < f.length; j++){
				s[this.inds[i]*IND.FPP + IND.FOR + j] += (i == 0 ? -1 : 1)*f[j];
			}
		}
	}

	get_buf_data(s){
		let data = [];
		for(let i = 0; i < this.inds.length; i++){
			for(let j = 0; j < 3; j++){
				data.push(s[this.inds[i]*IND.FPP + IND.POS + j]);
			}
		}
		return data;
	}
}

class SingleForcer{
	constructor(particle_ind, force){
		this.ind = particle_ind;
		this.f = force;

		this.data_len = 0;
	}

	set_force(force){
		this.f = force;
	}

	apply_force(s){
		for(let i = 0; i < this.f.length; i++){
			s[this.ind*IND.FPP + IND.FOR + i] += this.f[i];
		}
	}

}

class GravityForcer{
	constructor(val, num){
		this.g = val;
		this.num = num;

		this.data_len = 0;
	}

	apply_force(s){
		for(let n = 0; n < this.num; n++){
			let f = [0, 0, this.g*s[n*IND.FPP + IND.MAS]];
			for(let i = 0; i < f.length; i++){
				s[n*IND.FPP + IND.FOR + i] += f[i];
			}
		}
	}
}

class DragForcer{
	constructor(val, num){
		this.v = val;
		this.num = num;

		this.data_len = 0;
	}

	apply_force(s){
		for(let n = 0; n < this.num; n++){
			let v = s.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
			if(mag(v) == 0)
				v = APRX_0;
			let f = mult_scalar(mult_scalar(norm(v), -1), this.v*Math.pow(mag(v), 2));
			for(let i = 0; i < f.length; i++){
				s[n*IND.FPP + IND.FOR + i] += f[i];
			}
		}
	}
}

class FixConstraint{
	constructor(particle_ind, position){
		this.ind = particle_ind;
		this.p = position;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let i = 0; i < this.p.length; i++){
			s2[this.ind*IND.FPP + IND.POS + i] = this.p[i];
		}
		for(let i = 0; i < 3; i++){
			s2[this.ind*IND.FPP + IND.VEL + i] = 0;
		}
		for(let i = 0; i < 3; i++){
			s2[this.ind*IND.FPP + IND.FOR + i] = 0;
		}
	}
}

class WallConstraint{
	constructor(normal, orientation, center, width, height, coeff, num){
		this.n = norm(normal);
		this.dir = {
			x: norm(orientation),
			y: norm(cross3(normal, orientation))
		}
		this.p = center;
		this.w = width;
		this.h = height;
		this.constants = normal.concat([-normal[0]*center[0] - normal[1]*center[1] - normal[2]*center[2]]);
		this.c = -1*Math.abs(coeff);
		this.num = num;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p1 = s1.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			let p2 = s2.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			if(dist_point_plane(p1, this.constants) >= 0 && dist_point_plane(p2, this.constants) < 0){
				let p = add(p1, mult_scalar(this.n, -1*dist_point_plane(p1, this.constants)));
				let p_rel = sub(p, this.p);
				if(Math.abs(dot(p_rel, this.dir.x)) <= this.w && Math.abs(dot(p_rel, this.dir.y)) <= this.h){
					let v = s1.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
					let v_perp = mult_scalar(this.n, dot(v, this.n));
					v = add(sub(v, v_perp), mult_scalar(v_perp, this.c));
					for(let i = 0; i < 3; i++){
						s2[n*IND.FPP + IND.POS + i] = p[i];
						s2[n*IND.FPP + IND.VEL + i] = v[i];
					}
				}
			}
		}
	}
}

class AxisConstraint{
	constructor(axis_ind, offset, coeff, num){
		this.ind = axis_ind;
		this.off = offset;
		this.coeff = coeff;
		this.num = num;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			if((this.off > 0 && s1[n*IND.FPP + IND.POS + this.ind] <= this.off && s2[n*IND.FPP + IND.POS + this.ind] > this.off) || 
			   (this.off < 0 && s1[n*IND.FPP + IND.POS + this.ind] >= this.off && s2[n*IND.FPP + IND.POS + this.ind] < this.off)){
				s2[n*IND.FPP + IND.POS + this.ind] = this.off;
				s2[n*IND.FPP + IND.VEL + this.ind] = -this.coeff*s1[n*IND.FPP + IND.VEL + this.ind];
			}
		}
	}
}

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