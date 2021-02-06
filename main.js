let paused = false;
let init = {
	center: [0, 5, 10],
	size: 3,
	speed: 1
}

function main(){
	c = document.getElementById('canvas')
	setup_gl(c);

	cam = new CameraController([-9, -7, 9], [0, 0, 4], 1.25, .01);

	let grid_size = 75;
	let s = 1.0;
	let z = -.1;
	let sq = [
		0, 0, z, 0, 0, 0, 1,
		0, s, z, 0, 0, 0, 1,
		s, s, z, 0, 0, 0, 1,
		s, s, z, 0, 0, 0, 1,
		0, 0, z, 0, 0, 0, 1,
		s, 0, z, 0, 0, 0, 1
	];
	let sq_ind = 0;
	let grid = [];
	for(let x = -grid_size/2*s; x < grid_size/2*s; x += s){
		for(let y = -grid_size/2*s; y < grid_size/2*s; y += s, sq_ind++){
			let b = .2*(sq_ind % 2);
			let off = [x, y, 0, b, b, b, 0];
			for(let i = 0; i < sq.length; i++){
				grid.push(sq[i] + off[i % off.length]);
			}
		}
	}

	let num_particle = 90;
	F = [
		new BoidForcer(10, 10, 10, 2, num_particle),
		new DragForcer(.3, num_particle)
	];
	// for(let i = 0; i < num_particle - 1; i++){
	// 	F.push(new SpringForcer(.2, 100, 100, i, i + 1)); 
	// }
	C = [
		// new WallConstraint([0, -.2, 1], [1, 0, 0], [0, 3, 8], 2, 4, .5, num_particle),
		// new WallConstraint([0, .2, 1], [1, 0, 0], [0, -3, 6], 2, 4, .5, num_particle),
		// new WallConstraint([0, -.2, 1], [1, 0, 0], [0, 3, 4], 2, 4, .5, num_particle),
		// new WallConstraint([-.35, 0, 1], [0, 1, 0], [0, -3.5, 2], 2, 2, .5, num_particle),

		new AxisConstraint(0, -10, .85, num_particle),
		new AxisConstraint(0, 10, .85, num_particle),
		new AxisConstraint(1, -10, .85, num_particle),
		new AxisConstraint(1, 10, .85, num_particle),
		new AxisConstraint(2, -.05, .85, num_particle),
		new AxisConstraint(2, 20, .85, num_particle)
	];
	part_sys = new PartSys(num_particle);
	part_sys.init(init.center, init.size, init.speed, [5, 10], F, C);

	drawers = [
		new Drawer([0, 1, 1], [part_sys.num, part_sys.FC_num.tri, part_sys.FC_num.lin], [gl.POINTS, gl.TRIANGLES, gl.LINES]),
		new Drawer([1], [grid.length/7], [gl.TRIANGLES])
	];
	drawers[1].buffer_data(0, new Float32Array(grid));

	var model_matrix = new Matrix4();
	var view_matrix = new Matrix4();
	var proj_matrix = new Matrix4();
	view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
	proj_matrix.setPerspective(70, c.width/c.height, .01, 500);
	
	u_ModelMatrix = [];
	u_ViewMatrix = [];
	u_ProjMatrix = [];
	mvp_shaders = [0, 1];
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
			part_sys.applyAllForces(part_sys.s1, part_sys.F);
			part_sys.solver(timestep);
			part_sys.doConstraint(part_sys.s1, part_sys.s2, part_sys.C);
			part_sys.render(drawers[0]);
			part_sys.swap();
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
				part_sys.init(init.center, init.size, init.speed, [5, 10], F, C);
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

document.getElementById('launch_spd').value = init.speed.toFixed(1);
document.getElementById('launch_spd').onchange = function(){
	let value = parseFloat(this.value);
	if(!Number.isNaN(value)){
		init.speed = value;
	}
}

let tf_text = document.getElementById('tf_text');
let tf_range = document.getElementById('tf_range');

tf_text.onchange = function(){
	let value = parseFloat(this.value);
	if(!Number.isNaN(value)){
		value = value > tf_range.max ? tf_range.max : value;
		value = value < tf_range.min ? tf_range.min : value;
		part_sys.time_factor = value;
		tf_range.value = value
	}
}

tf_range.oninput = function(){
	part_sys.time_factor = this.value;
	tf_text.value = this.value;

}

tf_range.onchange = function(){
	if(Math.abs(this.value - 1.0) < .25)
		this.value = 1.0;
	part_sys.time_factor = this.value;
	tf_text.value = this.value;
}
