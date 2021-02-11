class SysHighlight{
	constructor(centers, bounds){
		this.planes = [];
		for(let i = 0; i < centers.length; i++){
			for(let j = 0; j < 6; j++){
				let n = [0, 0, 0];
				n[Math.floor(j/2)] = j % 2 == 0 ? -1 : 1;
				this.planes.push([n, add(centers[i], mult_scalar(n, bounds[i])), i]);
			}
		}
		this.boxes = [];
		let inds = [0, 1, 1, 3, 3, 2, 2, 0, 0, 4, 1, 5, 2, 6, 3, 7, 4, 5, 5, 7, 7, 6, 6, 4];
		let color = [1, 1, 0, 1];
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
		let collided = [];
		let cam_dir = norm(sub(cam_foc, cam_pos));
		for(let i = 0; i < this.planes.length; i++){
			if(check_ray_plane(this.planes[i][0], this.planes[i][1], cam_dir, cam_pos)){
				collided.push([dist(cam_pos, this.planes[i][1]), this.planes[i][2]]);
			}
		}
		if(collided.length == 0){
			this.data = new Float32Array(12*2*FPV);
		}
		let d = 10000000;
		let ind = -1;
		for(let i = 0; i < collided.length; i++){
			if(collided[i][0] < d){
				d = collided[i][0];
				this.data = this.boxes[collided[i][1]];
			}
		}
	}

	render(drawer){
		drawer.buffer_data(0, this.data);
	}
}