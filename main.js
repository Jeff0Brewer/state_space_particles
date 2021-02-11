let paused = false;
let fovy = 70;

function main(){
	c = document.getElementById('canvas');
	c.width = window.innerWidth;
	c.height = window.innerHeight;
	setup_gl(c);

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
	let boid_bound = 3.5;
	let boid_center = [-7, 7, boid_bound];
	let boid_sys = {
		num: boid_num,
		F: [
			new BoidForcer(8, 6, 8, 50, 1.5, 3, boid_center, 1.5, boid_num),
			new DragForcer(.3, boid_num)
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
			let s = [map(Math.random(), [0, 1], [40, 80])];
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

	let fire_num = 700;
	let fire_bound = 3.5;
	let fire_center = [-7, -7, 0];
	let fire_radius = 1.75;
	let fire_force = .25;
	let fire_map = function(val, bound){
		let colors = [[0, 0, 0], [.3, .3, .3], [.5, 0, 0], [1, 0, 0], [1, .5, 0]];
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
		let v = mult_scalar(norm([Math.random()*1 - .5, Math.random()*1 - .5, 1]), Math.random()*4 + 1);
		let f = [0, 0, 0];
		let m = map(Math.random(), [0, 1], [.01, .02]);
		let s = map(Math.random(), [0, 1], [25, 50]);
		let c = [1, 0, 0, 1];
		return p.concat(v, f, m, s, c);
	}
	let fire_sys = {
		num: fire_num,
		F: [
			new FireForcer(fire_center, fire_radius, 3, fire_force, fire_num),
			new GravityForcer(-9.8, fire_num),
			new DragForcer(.002, fire_num)
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

	let rad2 = Math.pow(2, .5);
	let tetra_num = 10;
	let spring_num = tetra_num*4;
	let spring_bound = 7;
	let spring_center = [7, 7, spring_bound];
	let spring_spawn = [-2, -4, 6];
	let spring_sys = {
		num: spring_num,
		F: [
			new GravityForcer(-9.8, spring_num),
			new DragForcer(.4, spring_num)
		],
		C: [
			new WallConstraint([.25, 0, 1], [0, 1, 0], add([-2.5, -4, 4], spring_center), 2, 3, .9, spring_num),
			new WallConstraint([0, .25, 1], [1, 0, 0], add([2.5, -4, 2], spring_center), 2, 3, .9, spring_num),
			new WallConstraint([-.4, 0, 1], [0, 1, 0], add([2.5, 1, 0], spring_center), 2, 3, .9, spring_num),

			new WallConstraint([0, 0, 1], [1, 1, 0], add([-2, 3, -5], spring_center), 2.5, 2.5, .9, spring_num),
			new WallConstraint([1, 1, 0], [0, 0, 1], add([-2 + 1.25*rad2, 3 + 1.25*rad2, -6], spring_center), 1, 2.5, .9, spring_num),
			new WallConstraint([-1, -1, 0], [0, 0, 1], add([-2 - 1.25*rad2, 3 - 1.25*rad2, -6], spring_center), 1, 2.5, .9, spring_num),
			new WallConstraint([-1, 1, 0], [0, 0, 1], add([-2 - 1.25*rad2, 3 + 1.25*rad2, -6], spring_center), 1, 2.5, .9, spring_num),
			new WallConstraint([1, -1, 0], [0, 0, 1], add([-2 + 1.25*rad2, 3 - 1.25*rad2, -6], spring_center), 1, 2.5, .9, spring_num),

			new SphereConstraint(add([-2.5, 3, -2], spring_center), 2, .9, spring_num),

			new AxisConstraint(0, -1, -spring_bound + spring_center[0], .9, spring_num),
			new AxisConstraint(0, 1, spring_bound + spring_center[0], .9, spring_num),
			new AxisConstraint(1, -1, -spring_bound + spring_center[1], .9, spring_num),
			new AxisConstraint(1, 1, spring_bound + spring_center[1], .9, spring_num),
			new AxisConstraint(2, -1, 0, .5, spring_num),
			new AxisConstraint(2, 1, 2*spring_bound, .5, spring_num)
		],
		init: function(){
			let p = add(add(spring_spawn, spring_center), [Math.random(), Math.random(), Math.random()]);
			let v = [0, 0, 0];
			let f = [0, 0, 0];
			let m = 5;
			let s = 0;
			let c = [1, 1, 1, 1];
			return p.concat(v, f, m, s, c);
		}
	}
	let connect_ind = [[0, 1], [1, 2], [2, 0], [0, 3], [1, 3], [2, 3]];
	for(let i = 0; i < tetra_num; i++){
		let spring_len = map(Math.random(), [0, 1], [.75, 1.25]);
		for(let j = 0; j < connect_ind.length; j++){
			spring_sys.F.push(new SpringForcer(spring_len, 10000, 100, connect_ind[j][0] + i*4, connect_ind[j][1] + i*4));
		}
	}

	let field_num = 1000;
	let field_bound = 5;
	let field_center = [7, -7, 0];
	let field_sys = {
		num: field_num,
		F: [
			new TornadoForcer(field_center, 1.5, 1, 5, 200, 20, field_num),
			new GravityForcer(-9.8, field_num),
			new DragForcer(.4, field_num)
		],
		C: [
			new AxisConstraint(0, -1, -field_bound + field_center[0], .95, field_num),
			new AxisConstraint(0, 1, field_bound + field_center[0], .95, field_num),
			new AxisConstraint(1, -1, -field_bound + field_center[1], .95, field_num),
			new AxisConstraint(1, 1, field_bound + field_center[1], .95, field_num),
			new AxisConstraint(2, -1, 0, .1, field_num),
			new AxisConstraint(2, 1, 2*field_bound, .95, field_num)
		],
		init: function(){
			let p = [map(Math.random(), [0, 1], [-field_bound, field_bound]),
					 map(Math.random(), [0, 1], [-field_bound, field_bound]),
					 map(Math.random(), [0, 1], [0, 2*field_bound])];
			p = add(field_center, p);
			let v = [0, 0, 0];
			let f = [0, 0, 0];
			let m = map(Math.random(), [0, 1], [1, 3]);
			let s = map(Math.random(), [0, 1], [50, 150]);
			let shade = map(Math.random(), [0, 1], [.3, .7]);
			let c = [shade, shade, shade + map(Math.random(), [0, 1], [.25, .5])];
			return p.concat(v, f, m, s, c);
		}
	}

	cam = new CameraController([-17, -5, 8], [0, 0, 3], 1.25, .01);
	sys_highlight = new SysHighlight([spring_center, add(field_center, [0, 0, field_bound]), add(fire_center, [0, 0, fire_bound]), boid_center], [spring_bound, field_bound, fire_bound, boid_bound]);

	part_sys = [
		new PartSys(boid_sys.num, boid_sys.F, boid_sys.C, boid_sys.init),
		new PartSys(fire_sys.num, fire_sys.F, fire_sys.C, fire_sys.init),
		new PartSys(spring_sys.num, spring_sys.F, spring_sys.C, spring_sys.init),
		new PartSys(field_sys.num, field_sys.F, field_sys.C, field_sys.init)
	];
	for(let i = 0; i < part_sys.length; i++){
		part_sys[i].init();
	}

	drawers = [
		new Drawer([2, 0, 0], [part_sys[0].num, part_sys[0].FC_num.tri, part_sys[0].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([3, 0, 0], [part_sys[1].num, part_sys[1].FC_num.tri, part_sys[1].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([1, 0, 0], [part_sys[2].num, part_sys[2].FC_num.tri, part_sys[2].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([1, 0, 0], [part_sys[3].num, part_sys[3].FC_num.tri, part_sys[3].FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([0], [grid.length/FPV], [gl.TRIANGLES]),
		new Drawer([0], [sys_highlight.data.length/FPV], [gl.LINES])
	];
	drawers[drawers.length - 2].buffer_data(0, new Float32Array(grid));

	model_matrix = new Matrix4();
	view_matrix = new Matrix4();
	proj_matrix = new Matrix4();
	view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
	proj_matrix.setPerspective(fovy, c.width/c.height, .01, 500);
	
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
				part_sys[i].solver(timestep);
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

		sys_highlight.update(cam.pos, cam.foc);
		sys_highlight.render(drawers[drawers.length - 1]);

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

document.getElementById('solver_menu').onchange = function(){
	for(let i = 0; i < part_sys.length; i++){
		part_sys[i].set_solver(this.value);
	}
}

document.body.onresize = function(){
	c.width = window.innerWidth;
	c.height = window.innerHeight;
	if(gl){
		gl.viewport(0, 0, c.width, c.height);
		proj_matrix.setPerspective(fovy, c.width/c.height, .01, 500);
		for(let i = 0; i < mvp_shaders.length; i++){
			switch_shader(mvp_shaders[i]);
			gl.uniformMatrix4fv(gl.getUniformLocation(gl.program, 'u_ProjMatrix'), false, proj_matrix.elements);
		}
	}
}