class FireConstraint{
	constructor(lifetime, max_mass, max_size, color_map, init, num){
		this.num = num;
		this.lif = lifetime;
		this.mass = max_mass;
		this.size = max_size;
		this.color_map = color_map;
		this.init = init;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let lif = s2[n*IND.FPP + IND.LIF];
			if(lif > this.lif){
				let p_new = this.init();
				for(let i = 0; i < p_new.length; i++){
					s2[n*IND.FPP + i] = p_new[i];
				}
			}
			else{
				let color = this.color_map(lif, [0, this.lif]);
				s2[n*IND.FPP + IND.MAS] = map(lif, [0, this.lif], [this.mass, 0]);
				s2[n*IND.FPP + IND.SIZ]= map(lif, [0, this.lif], [this.size, 0]);
				for(let i = 0; i < color.length; i++){
					s2[n*IND.FPP + IND.COL + i] = color[i];
				}
			}
		}
	}
}

class SphereConstraint{
	constructor(center, radius, coeff, num){
		this.num = num;
		this.c = center;
		this.r = radius;
		this.coeff = -1*Math.abs(coeff);


		let iso = gen_iso(1, 'LIN');
		let lin = [];
		let color = [1, 1, 1, 1];
		for(let i = 0; i < iso.length; i++){
			iso[i] = add(mult_scalar(iso[i], this.r), this.c);
			lin = lin.concat(iso[i]);
			lin = lin.concat(color);
			lin.push(0);
		}

		this.data = [[], lin];
		this.data_len = [0, lin.length];
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p2 = s2.slice(n*IND.FPP + IND.POS, n*IND.FPP + IND.POS + 3);
			let v2 = s2.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
			if(dist(p2, this.c) < this.r && dot(sub(p2, this.c), v2) < 0){
				let v2 = s2.slice(n*IND.FPP + IND.VEL, n*IND.FPP + IND.VEL + 3);
				let dir = norm(sub(p2, this.c));
				let v_perp = mult_scalar(dir, dot(v2, dir));
				v2 = add(sub(v2, v_perp), mult_scalar(v_perp, this.coeff));
				p2 = add(mult_scalar(dir, this.r), this.c);
				for(let i = 0; i < 3; i++){
					s2[n*IND.FPP + IND.POS + i] = p2[i];
					s2[n*IND.FPP + IND.VEL + i] = v2[i];
				}
			}
		}
	}

	get_buf_data(s){
		return this.data;
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
		let color = [1, 1, 1, 1];
		let ind = [0, 1, 1, 2, 2, 3, 3, 0];
		for(let i = 0; i < ind.length; i++){
			lin = lin.concat(points[ind[i]]);
			lin = lin.concat(color);
			lin.push(0);
		}

		this.data = [[], lin];
		this.data_len = [0, lin.length];
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

class BoundConstraint{
	constructor(axis_ind, min, max, num){
		this.ind = axis_ind;
		this.min = min;
		this.max = max;
		this.siz = max - min;
		this.num = num;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p1 = s1[n*IND.FPP + IND.POS + this.ind];
			let p2 = s2[n*IND.FPP + IND.POS + this.ind];
			if(p1 >= this.min && p2 < this.min){
				s2[n*IND.FPP + IND.POS + this.ind] += this.siz;
			}
			else if(p1 <= this.max && p2 > this.max){
				s2[n*IND.FPP + IND.POS + this.ind] -= this.siz;
			}
		}
	}
}


class AxisConstraint{
	constructor(axis_ind, dir, offset, coeff, num){
		this.ind = axis_ind;
		this.dir = dir > 0 ? 1 : -1;
		this.off = offset;
		this.coeff = -1*Math.abs(coeff);
		this.num = num;

		this.data_len = 0;
	}

	constrain(s1, s2){
		for(let n = 0; n < this.num; n++){
			let p1 = s1[n*IND.FPP + IND.POS + this.ind];
			let p2 = s2[n*IND.FPP + IND.POS + this.ind];
			if((this.dir > 0 && p1 <= this.off && p2 > this.off) || 
			   (this.dir < 0 && p1 >= this.off && p2 < this.off)){
				s2[n*IND.FPP + IND.POS + this.ind] = this.off;
				s2[n*IND.FPP + IND.VEL + this.ind] = this.coeff*s2[n*IND.FPP + IND.VEL + this.ind];
			}
		}
	}
}
