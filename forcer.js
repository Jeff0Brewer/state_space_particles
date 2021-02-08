class FireForcer{
	constructor(center, radius, height, heat_force, num){
		this.num = num
		this.c = center;
		this.r = radius;
		this.h = height;
		this.f = heat_force;

		this.data_len = 0;
	}

	apply_force(s){
		for(let n = 0; n < this.num; n++){
			let p = s.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			p = sub(p, this.c);
			let heat_mag = map(p[2], [0, this.h], [map(mag(p.slice(0, 2)), [0, this.r], [this.f, 0]), 0]);
			heat_mag = heat_mag > 0 ? heat_mag : 0;
			let f = mult_scalar([0, 0, 1], heat_mag);
			for(let i = 0; i < f.length; i++){
				s[n*IND.FPP + IND.FOR + i] += f[i];
			}
		}
	}
}

class BoidForcer{
	constructor(co_f, al_f, sp_f, av_f, d, center, radius, num){
		this.num = num;
		this.c = center;
		this.r = radius;
		this.d = d;
		this.f = {
			co: co_f,
			al: al_f,
			sp: sp_f,
			av: av_f
		};

		let iso = gen_iso(2);
		let tri = [];
		let color = [1, 1, 1, 1];
		for(let i = 0; i < iso.length; i++){
			iso[i] = add(mult_scalar(iso[i], this.r), this.c);
			tri = tri.concat(iso[i]);
			tri = tri.concat(color);
			tri.push(0);
		}

		this.data = [tri, []];
		this.data_len = [tri.length, 0];
	}

	apply_force(s){
		let p = [];
		let v = [];
		for(let n = 0; n < this.num; n++){
			let f = [0, 0, 0];
			let co = [0, 0, 0];
			let al = [0, 0, 0];
			let sp = [0, 0, 0];
			let av = [0, 0, 0];
			let count = 0;
			for(let i = 0; i < this.num; i++){
				if(n == 0){
					p.push(s.slice(i*IND.FPP + IND.POS, i*IND.FPP + IND.POS + 3));
					v.push(s.slice(i*IND.FPP + IND.VEL, i*IND.FPP + IND.VEL + 3));
				}
				let p_diff = sub(p[n], p[i]);
				let d = mag(p_diff);
				if(d < this.d && i != n){
					count++;
					co = add(co, p[i]);
					al = add(al, v[i]);
					sp = add(sp, mult_scalar(norm(p_diff), map(d, [0, this.d], [this.f.sp, 0])));
				}
			}
			av = sub(p[n], this.c);
			let d_sph = mag(av) - this.r;
			if(d_sph < this.d){
				av = mult_scalar(norm(av), this.f.av*(this.d - d_sph)/this.d);
				f = add(f, av);
			}
			if(count > 0){
				co = mult_scalar(co, 1/count);
				co = mult_scalar(norm(sub(co, p[n])), this.f.co);
				sp = mult_scalar(norm(sp), this.f.sp);
				al = mult_scalar(norm(sub(al, v[n])), this.f.al);
				for(let j = 0; j < 3; j++){
					f[j] += co[j] + al[j] + sp[j]; 
				}
			}
			for(let i = 0; i < f.length; i++){
				s[n*IND.FPP + IND.FOR + i] += f[i];
			}
		}
	}

	get_buf_data(s){
		return this.data;
	}
}

class SpringForcer{
	constructor(length, strength, damping, ind_a, ind_b){
		this.len = length;
		this.str = strength;
		this.dmp = damping;
		this.inds = [ind_a, ind_b];

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
			lin.push(0);
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
				let f = mult_scalar(norm(v), this.v*Math.pow(mag(v), 2)*s[n*IND.FPP + IND.MAS]);
				for(let i = 0; i < f.length; i++){
					if(!isFinite(f[i]))
						f[i] = Math.sign(f[i])*this.max;
					s[n*IND.FPP + IND.FOR + i] += f[i];
				}
			}
		}
	}
}