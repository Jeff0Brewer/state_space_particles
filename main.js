let paused = false;
let time_factor = 1.0;

function main(){
	c = document.getElementById('canvas')
	setup_gl(c);

	cam = new CameraController([-15, 0, 7], [0, 0, 3], 1.25, .01);

	let grid_size = 75;
	let s = 1.0;
	let z = -.1;
	let sq = [
		0, 0, z, 0, 0, 0, 1, 0,
		0, s, z, 0, 0, 0, 1, 0,
		s, s, z, 0, 0, 0, 1, 0,
		s, s, z, 0, 0, 0, 1, 0,
		0, 0, z, 0, 0, 0, 1, 0,
		s, 0, z, 0, 0, 0, 1, 0
	];
	let sq_ind = 0;
	let grid = [];
	for(let x = -grid_size/2*s; x < grid_size/2*s; x += s){
		for(let y = -grid_size/2*s; y < grid_size/2*s; y += s, sq_ind++){
			let b = .2*(sq_ind % 2);
			let off = [x, y, 0, b, b, b, 0, 0];
			for(let i = 0; i < sq.length; i++){
				grid.push(sq[i] + off[i % off.length]);
			}
		}
	}

	let boid_num = 90;
	let boid_bound = 4;
	let boid_center = [0, 3.5, boid_bound];
	let boid_sys = {
		num: boid_num,
		F: [
			new BoidForcer(8, 6, 8, 50, 1.5, boid_center, 1.5, boid_num),
			new DragForcer(.1, boid_num)
		],
		C: [
			new BoundConstraint(0, -boid_bound + boid_center[0], boid_bound + boid_center[0], boid_num),
			new BoundConstraint(1, -boid_bound + boid_center[1], boid_bound + boid_center[1], boid_num),
			new AxisConstraint(2, -1, 0, 1, boid_num),
			new AxisConstraint(2, 1, 2*boid_bound, 1, boid_num)
		],
		init: function(){
			let p = [];
			let v = [];
			let f = [];
			let m = [map(Math.random(), [0, 1], [5, 10])];
			let s = [map(Math.random(), [0, 1], [25, 50])];
			let c = [.5, .5, .5, 1];
			c[Math.floor(Math.random()*3)] = 1;
			for(let i = 0; i < 3; i++){
				p.push(map(Math.random(), [0, 1], [-boid_bound, boid_bound]) + boid_center[i]);
				v.push(map(Math.random(), [0, 1], [-2, 2]));
				f.push(0);
			}
			return p.concat(v, f, m, s, c);
		}
	};

	let fire_num = 650;
	let fire_bound = 3.5;
	let fire_center = [0, 12, 0];
	let fire_radius = 2;
	let fire_force = .4;
	let fire_map = function(val, bound){
		let colors = [[0, 0, 0], [.2, .2, .2], [.5, 0, 0], [1, 0, 0], [1, .5, 0]];
		let per = (val - bound[0])/(bound[1] - bound[0]);
		if(per < 0)
			return colors[0];
		if(per > 1)
			return colors[colors.length - 1];
		let ind = Math.floor(per * (colors.length - 1));
		per = Math.abs(ind - per)*colors.length;
		let color = [];
		for(let i = 0; i < 3; i++){
			color.push(map(per, [0, 1], [colors[ind][i], colors[ind + 1][i]]));
		}
		return color;
	}
	let fire_init = function(){
		let angle = Math.random()*2*Math.PI;
		let radius = Math.random()*fire_radius;
		let p = add(fire_center, [Math.cos(angle)*radius, Math.sin(angle)*radius, 0]);
		let v = mult_scalar(norm([Math.random()*1 - .5, Math.random()*1 - .5, 1]), Math.random()*5);
		let f = [0, 0, 0];
		let m = map(Math.random(), [0, 1], [.05, .1]);
		let s = map(Math.random(), [0, 1], [25, 50]);
		let c = [1, 0, 0, 1];
		return p.concat(v, f, m, s, c);
	}
	let fire_sys = {
		num: fire_num,
		F: [
			new FireForcer(fire_center, fire_radius, 3, fire_force, fire_num),
			new GravityForcer(-9.8, fire_num),
			new DragForcer(.4, fire_num)
		],
		C: [
			new FireConstraint(fire_center, fire_radius, fire_force, fire_map, fire_init, fire_num),
			new AxisConstraint(0, -1, -fire_bound + fire_center[0], .5, fire_num),
			new AxisConstraint(0, 1, fire_bound + fire_center[0], .5, fire_num),
			new AxisConstraint(1, -1, -fire_bound + fire_center[1], .5, fire_num),
			new AxisConstraint(1, 1, fire_bound + fire_center[1], .5, fire_num),
			new AxisConstraint(2, -1, 0, .5, fire_num),
			new AxisConstraint(2, 1, 2*fire_bound, .5, fire_num)
		],
		init: fire_init
	}


	part_sys = [
		new PartSys(boid_sys.num, boid_sys.F, boid_sys.C, boid_sys.init),
		new PartSys(fire_sys.num, fire_sys.F, fire_sys.C, fire_sys.init)
	];
	for(let i = 0; i < part_sys.length; i++){
		part_sys[i].init();
	}

	drawers = [
		new Drawer([2, 0, 0], [part_sys[0].num, part_sys[0].FC_num.tri, part_sys[0].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([3, 0, 0], [part_sys[1].num, part_sys[1].FC_num.tri, part_sys[1].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([0], [grid.length/FPV], [gl.TRIANGLES])
	];
	drawers[drawers.length - 1].buffer_data(0, new Float32Array(grid));

	var model_matrix = new Matrix4();
	var view_matrix = new Matrix4();
	var proj_matrix = new Matrix4();
	view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
	proj_matrix.setPerspective(70, c.width/c.height, .01, 500);
	
	u_ModelMatrix = [];
	u_ViewMatrix = [];
	u_ProjMatrix = [];
	mvp_shaders = [0, 1, 2, 3];
	for(let i = 0; i < mvp_shaders.length; i++){
		switch_shader(mvp_shaders[i]);
		u_ModelMatrix.push(gl.getUniformLocation(gl.program, 'u_ModelMatrix'));
		u_ViewMatrix.push(gl.getUniformLocation(gl.program, 'u_ViewMatrix'));
		u_ProjMatrix.push(gl.getUniformLocation(gl.program, 'u_ProjMatrix'));
		
		gl.uniformMatrix4fv(u_ModelMatrix[i], false, model_matrix.elements);
		gl.uniformMatrix4fv(u_ViewMatrix[i], false, view_matrix.elements);
		gl.uniformMatrix4fv(u_ProjMatrix[i], false, proj_matrix.elements);
	}

	let timestep = 1000/60;
	var tick = function(){
		if(!paused){
			for(let i = 0; i < part_sys.length; i++){
				part_sys[i].applyAllForces(part_sys[i].s1, part_sys[i].F);
				part_sys[i].solver(timestep*time_factor);
				part_sys[i].doConstraint(part_sys[i].s1, part_sys[i].s2, part_sys[i].C);
				part_sys[i].render(drawers[i]);
				part_sys[i].swap();
			}
		}
		cam.strafe(timestep);
		view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
		for(let i = 0; i < mvp_shaders.length; i++){
			switch_shader(mvp_shaders[i]);
			gl.uniformMatrix4fv(u_ViewMatrix[i], false, view_matrix.elements);
		}

		draw();
	}
	setInterval(tick, timestep);

	window.addEventListener('keydown', key_down, false);
	window.addEventListener('keyup', key_up, false);

	c.onmousedown = function(e){
		cam.mousedown(e);
	}

	c.onmousemove = function(e){
		cam.mousemove(e)
	}

	c.onmouseup = function(e){
		cam.mouseup(e);
	}

	c.onwheel = function(e){
		cam.wheel(e);
	}
}

function draw(){
	gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

	for(let i = 0; i < drawers.length; i++){
		drawers[i].draw();
	}
}

function key_down(e){
	let char = String.fromCharCode(e.keyCode);
	switch(char){
		case 'W':
			cam.add_strafe([0, 1]);
			break;
		case 'A':
			cam.add_strafe([-1, 0]);
			break;
		case 'S':
			cam.add_strafe([0, -1]);
			break;
		case 'D':
			cam.add_strafe([1, 0]);
			break;
		case 'R':
			if(!paused)
				for(let i = 0; i < part_sys.length; i++){
					part_sys[i].init();
				}
			break;
		case 'P':
			paused = !paused;
			break;
	}
}

function key_up(e){
	let char = String.fromCharCode(e.keyCode);
	switch(char){
		case 'W':
			cam.add_strafe([0, -1]);
			break;
		case 'A':
			cam.add_strafe([1, 0]);
			break;
		case 'S':
			cam.add_strafe([0, 1]);
			break;
		case 'D':
			cam.add_strafe([-1, 0]);
			break;
	}
}

let tf_text = document.getElementById('tf_text');
let tf_range = document.getElementById('tf_range');

tf_text.onchange = function(){
	let value = parseFloat(this.value);
	if(!Number.isNaN(value)){
		value = value > tf_range.max ? tf_range.max : value;
		value = value < tf_range.min ? tf_range.min : value;
		tf_range.value = value;
		time_factor = value;
	}
}

tf_range.oninput = function(){
	tf_text.value = this.value;
	time_factor = this.value;
}

tf_range.onchange = function(){
	if(Math.abs(this.value - 1.0) < .25)
		this.value = 1.0;
	tf_text.value = this.value;
	time_factor = this.value;
}
