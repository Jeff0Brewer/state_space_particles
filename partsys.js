IND = { // px, py, pz, vx, vy, vz, fx, fy, fz, m
	POS: 0, //position
	VEL: 3, //velocity
	FOR: 6, //force
	MAS: 9, //mass
	SIZ: 10, //size
	COL: 11, //color
	FPP: 15 //floats per particle
}
APRX_0 = [0, 0, .00000000000001];
FPV = 8;

function PartSys(num, F, C, init){
	this.num = num;
	this.F = F;
	this.C = C;
	this.init_single = init;
	this.s1 = new Float32Array(IND.FPP*num);
	this.s2 = new Float32Array(IND.FPP*num);

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

	this.init = function(){
		for(let i = 0; i < this.num; i++){
			let p_state = this.init_single();
			for(let j = 0; j < IND.FPP; j++){
				this.s1[i*IND.FPP + j] = p_state[j];
			}
			this.s2.set(this.s1);
		}
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
			buf[buf_ind] = this.s2[n*IND.FPP + IND.SIZ];
			buf_ind++;
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