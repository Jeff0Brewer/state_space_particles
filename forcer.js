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