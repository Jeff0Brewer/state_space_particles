class SysHighlight{
	constructor(centers, bounds, switch_function){
		this.c = centers;
		this.b = bounds;
		this.switch_func = switch_function;
		this.ind = -1;
		this.boxes = [];
		let inds = [0, 1, 1, 3, 3, 2, 2, 0, 0, 4, 1, 5, 2, 6, 3, 7, 4, 5, 5, 7, 7, 6, 6, 4];
		let color = [1, 1, 1, .25];
		let size = 0;
		for(let i = 0; i < centers.length; i++){
			let points = [
				add(centers[i], [bounds[i], bounds[i], bounds[i]]),
				add(centers[i], [bounds[i], bounds[i], -bounds[i]]),
				add(centers[i], [bounds[i], -bounds[i], bounds[i]]),
				add(centers[i], [bounds[i], -bounds[i], -bounds[i]]),
				add(centers[i], [-bounds[i], bounds[i], bounds[i]]),
				add(centers[i], [-bounds[i], bounds[i], -bounds[i]]),
				add(centers[i], [-bounds[i], -bounds[i], bounds[i]]),
				add(centers[i], [-bounds[i], -bounds[i], -bounds[i]])
			];
			this.boxes.push(new Float32Array(inds.length*FPV));
			let buf_ind = 0;
			for(let j = 0; j < inds.length; j++){
				for(let l = 0; l < points[inds[j]].length; l++, buf_ind++){
					this.boxes[this.boxes.length - 1][buf_ind] = points[inds[j]][l];
				}
				for(let l = 0; l < color.length; l++, buf_ind++){
					this.boxes[this.boxes.length - 1][buf_ind] = color[l];
				}
				this.boxes[this.boxes.length - 1][buf_ind] = size;
				buf_ind++;
			}
		}

		this.data = new Float32Array(12*2*FPV);
	}

	update(cam_pos, cam_foc){
		let last_ind = this.ind;
		let collided = [];
		for(let i = 0; i < this.c.length; i++){
			let d_c = dist_point_line(this.c[i], cam_pos, cam_foc);
			if(d_c < this.b[i]*1.5){
				collided.push([dist(this.c[i], cam_pos), i]);
			}
		}
		let d = 100000000;
		for(let i = 0; i < collided.length; i++){
			if(collided[i][0] < d){
				d = collided[i][0];
				this.ind = collided[i][1];
				this.data = this.boxes[this.ind];
			}
		}
		if(this.ind != last_ind){
			this.switch_func(this.ind);
		}
	}

	render(drawer){
		drawer.buffer_data(0, this.data);
	}
}