IND = { // px, py, pz, vx, vy, vz, fx, fy, fz, m
	POS: 0, //position
	VEL: 3, //velocity
	FOR: 6, //force
	MAS: 9, //mass
	COL: 12, //color
	FPP: 16 //floats per particle
}
APRX_0 = [0, 0, .00000000000001];
FPV = 7;

function PartSys(num){
	this.num = num;
	this.time_factor = 1.0;

	this.s1 = new Float32Array(IND.FPP*num);
	this.s2 = new Float32Array(IND.FPP*num);

	this.F = [];
	this.C = [];

	this.init = function(center, size, vel, m_b, F, C){
		this.F = F;
		this.C = C;

		let colors = [[1, .5, .5, 1], [.5, 1, .5, 1], [.5, .5, 1, 1]];
		let p_b = [[-size, size], [-size, size], [-size, size]];
		for(let i = 0; i < this.s1.length; i++){
			let p_ind = Math.floor(i / IND.FPP);
			let v_ind = i % IND.FPP;
			if(v_ind < IND.POS + 3)
				this.s1[i] = map(Math.random(), [0, 1], p_b[v_ind]) + center[v_ind - IND.POS];
			else if (v_ind < IND.VEL + 3)
				this.s1[i] = (Math.random() > .5 ? -1 : 1)*vel*Math.random();
			else if (v_ind < IND.FOR + 3)
				this.s1[i] = 0;
			else if (v_ind == IND.MAS)
				this.s1[i] = map(Math.random(), [0, 1], m_b);
			else if (v_ind < IND.COL + 4)
				this.s1[i] = colors[p_ind % colors.length][v_ind - IND.COL];

			this.s2[i] = this.s1[i];
		}

		let FC_tri_len = 0;
		let FC_lin_len = 0;
		for(let i = 0; i < this.F.length; i++){
			FC_tri_len += this.F[i].data_len != 0 ? this.F[i].data_len[0] : 0;
			FC_lin_len += this.F[i].data_len != 0 ? this.F[i].data_len[1] : 0;
		}
		for(let i = 0; i < this.C.length; i++){
			FC_tri_len += this.C[i].data_len != 0 ? this.C[i].data_len[0] : 0;
			FC_lin_len += this.C[i].data_len != 0 ? this.C[i].data_len[1] : 0;
		}
		this.FC_num = {
			tri: FC_tri_len/FPV,
			lin: FC_lin_len/FPV,
			all: (FC_tri_len + FC_lin_len)/FPV
		};
	}

	this.applyAllForces = function(s, F){
		for(let n = 0; n < this.num; n++){
			for(let i = 0; i < 3; i++){
				s[n*IND.FPP + IND.FOR + i] = 0;
			}
		}
		for(let i = 0; i < F.length; i++){
			F[i].apply_force(s);
		}
	}

	this.dotFinder = function(s){
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

	this.solver = function(elapsed){
		elapsed *= this.time_factor;
		// this.s2 = this.euler_e(this.s1, this.s2, elapsed);
		// this.s2 = this.midpoint_e(this.s1, this.s2, elapsed);
		// this.s2 = this.euler_i(this.s1, this.s2, elapsed);
		this.s2 = this.midpoint_i(this.s1, this.s2, elapsed);
	}

	this.euler_e = function(s1, s2, elapsed){
		let out = new Float32Array(IND.FPP*this.num);
		let s1dot = this.dotFinder(s1);
		for(let i = 0; i < out.length; i++){
			out[i] = s1[i] + s1dot[i]*elapsed/1000;
		}
		return out;
	}

	this.midpoint_e = function(s1, s2, elapsed){
		let out = new Float32Array(IND.FPP*this.num);
		let sMdot = this.dotFinder(this.euler_e(s1, s2, elapsed*.5));
		for(let i = 0; i < out.length; i++){
			out[i] = s1[i] + sMdot[i]*elapsed/1000;
		}
		return out;
	}

	this.euler_i = function(s1, s2, elapsed, explicit_solver){
		let out = new Float32Array(IND.FPP*this.num);
		let s2_0 = this.euler_e(s1, s2, elapsed);
		let s3_0 = this.euler_e(s2_0, s2, -elapsed);
		for(let i = 0; i < out.length; i++){
			out[i] = s2_0[i] - .5*(s1[i] - s3_0[i]);
		}
		return out;
	}

	this.midpoint_i = function(s1, s2, elapsed, explicit_solver){
		let out = new Float32Array(IND.FPP*this.num);
		let s2_0 = this.midpoint_e(s1, s2, elapsed);
		let s3_0 = this.midpoint_e(s2_0, s2, -elapsed);
		for(let i = 0; i < out.length; i++){
			out[i] = s2_0[i] - .5*(s1[i] - s3_0[i]);
		}
		return out;
	}

	this.doConstraint = function(s1, s2, C){
		for(let i = 0; i < C.length; i++){
			C[i].constrain(s1, s2);
		}
	}

	this.render = function(drawer){
		let buf = new Float32Array((this.num + this.FC_num.all)*FPV);
		let buf_ind = 0;
		for(let n = 0; n < this.num; n++){
			for(let i = 0; i < 3; i++, buf_ind++){
				buf[buf_ind] = this.s2[n*IND.FPP + IND.POS + i];
			}
			for(let i = 0; i < 4; i++, buf_ind++){
				buf[buf_ind] = this.s2[n*IND.FPP + IND.COL + i];
			}
		}

		let FC = this.F.concat(this.C);
		let tri_ind = this.num*FPV;
		let lin_ind = this.num*FPV + this.FC_num.tri*FPV;
		for(let i = 0; i < FC.length; i++){
			let data = FC[i].data_len != 0 ? FC[i].get_buf_data(this.s2) : [[], []];
			for(let j = 0; j < data[0].length; j++, tri_ind++){
				buf[tri_ind] = data[0][j];
			}
			for(let j = 0; j < data[1].length; j++, lin_ind++){
				buf[lin_ind] = data[1][j];
			}
		}
		
		drawer.buffer_data(0, buf);
	}

	this.swap = function(){
		this.s1.set(this.s2);
	}
}

class SpringForcer{
	constructor(length, strength, damping, ind_a, ind_b){
		this.len = length;
		this.str = strength;
		this.dmp = damping;
		this.inds = [ind_a, ind_b];
		this.max = 100000;

		this.data_len = [0, FPV*2];
	}

	apply_force(s){
		let pos_diff = sub(s.slice(this.inds[0]*IND.FPP + IND.POS, this.inds[0]*IND.FPP + IND.POS + 3), s.slice(this.inds[1]*IND.FPP + IND.POS, this.inds[1]*IND.FPP + IND.POS + 3));
		let vel_diff = sub(s.slice(this.inds[0]*IND.FPP + IND.VEL, this.inds[0]*IND.FPP + IND.VEL + 3), s.slice(this.inds[1]*IND.FPP + IND.VEL, this.inds[1]*IND.FPP + IND.VEL + 3));
		if(mag(pos_diff) == 0)
			pos_diff = APRX_0.slice();
		if(mag(vel_diff) == 0)
			vel_diff = APRX_0.slice();
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
		let lin = [];
		let color = [1, 1, 1, .5];
		for(let i = 0; i < this.inds.length; i++){
			for(let j = 0; j < 3; j++){
				lin.push(s[this.inds[i]*IND.FPP + IND.POS + j]);
			}
			for(let j = 0; j < 4; j++){
				lin.push(color[j]);
			}
		}
		return [[], lin];
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
		this.v = -1*Math.abs(val);
		this.num = num;
		this.max = 10000;

		this.data_len = 0;
	}

	apply_force(s){
		for(let n = 0; n < this.num; n++){
			let v = s.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
			if(mag(v) > 0){
				let f = mult_scalar(norm(v), this.v*Math.pow(mag(v), 2));
				for(let i = 0; i < f.length; i++){
					if(!isFinite(f[i]))
						f[i] = Math.sign(f[i])*this.max;
					s[n*IND.FPP + IND.FOR + i] += f[i];
				}
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

		let points = [
			add(add(this.p, mult_scalar(this.dir.x, this.w)), mult_scalar(this.dir.y, this.h)),
			add(add(this.p, mult_scalar(this.dir.x, this.w)), mult_scalar(this.dir.y, -this.h)),
			add(add(this.p, mult_scalar(this.dir.x, -this.w)), mult_scalar(this.dir.y, -this.h)),
			add(add(this.p, mult_scalar(this.dir.x, -this.w)), mult_scalar(this.dir.y, this.h))
		];
		let lin = [];
		let color = [1, 1, 1, .5];
		let ind = [0, 1, 1, 2, 2, 3, 3, 0];
		for(let i = 0; i < ind.length; i++){
			lin = lin.concat(points[ind[i]]);
			lin = lin.concat(color);
		}
		this.data = [[], lin];
		this.data_len = [0, FPV*ind.length];
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p1 = s1.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			let p2 = s2.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			if(dist_point_plane(p1, this.constants) >= 0 && dist_point_plane(p2, this.constants) < 0){
				let p = add(p1, mult_scalar(this.n, dist_point_plane(p1, this.constants)));
				let p_rel = sub(p, this.p);
				if(Math.abs(dot(p_rel, this.dir.x)) <= this.w && Math.abs(dot(p_rel, this.dir.y)) <= this.h){
					let v = s2.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
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

	get_buf_data(s){
		return this.data;
	}
}

class AxisConstraint{
	constructor(axis_ind, offset, coeff, num){
		this.ind = axis_ind;
		this.off = offset;
		this.coeff = -1*Math.abs(coeff);
		this.num = num;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p1 = s1[n*IND.FPP + IND.POS + this.ind];
			let p2 = s2[n*IND.FPP + IND.POS + this.ind];
			if((this.off > 0 && p1 <= this.off && p2 > this.off) || 
			   (this.off < 0 && p1 >= this.off && p2 < this.off)){
				s2[n*IND.FPP + IND.POS + this.ind] = this.off;
				s2[n*IND.FPP + IND.VEL + this.ind] = this.coeff*s2[n*IND.FPP + IND.VEL + this.ind];
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