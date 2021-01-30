let paused = false;
let init = {
	center: [0, 5, 10],
	size: .5,
	speed: 1
}

function main(){
	c = document.getElementById('canvas')
	var gl = getWebGLContext(c);
	initShaders(gl, document.getElementById('v_ball').text, document.getElementById('f_ball').text);
	gl.enable(gl.DEPTH_TEST);
	gl.clearColor(0, 0, 0, 1);

	cam = new CameraController([-15, -10, 5], [0, 0, 2], 1.25, .01);

	let num_particle = 5000;
	F = [
		new GravityForcer(-9.8, num_particle),
		new DragForcer(.5, num_particle)
	];
	// for(let i = 0; i < num_particle - 1; i++){
	// 	F.push(new SpringForcer(.25, 100, 100, i, i + 1)); 
	// }
	C = [
		new WallConstraint([0, -.2, 1], [1, 0, 0], [0, 5, 8], 2, 5, .5, num_particle),
		new WallConstraint([0, .2, 1], [1, 0, 0], [0, -5, 6], 2, 5, .5, num_particle),
		new WallConstraint([0, -.2, 1], [1, 0, 0], [0, 5, 4], 2, 5, .5, num_particle),
		new WallConstraint([0, .2, 1], [1, 0, 0], [0, -5, 2], 2, 5, .5, num_particle),


		new WallConstraint([1, 0, 0], [0, 1, 0], [-10, 0, 0], 100, 100, .85, num_particle),
		new WallConstraint([-1, 0, 0], [0, -1, 0], [10, 0, 0], 100, 100, .85, num_particle),
		new WallConstraint([0, 1, 0], [1, 0, 0], [0, -10, 0], 100, 100, .85, num_particle),
		new WallConstraint([0, -1, 0], [-1, 0, 0], [0, 10, 0], 100, 100, .85, num_particle),
		new WallConstraint([0, 0, 1], [1, 0, 0], [0, 0, 0], 100, 100, .85, num_particle),
		new WallConstraint([0, 0, -1], [-1, 0, 0], [0, 0, 15], 100, 100, .85, num_particle)
	];
	part_sys = new PartSys(num_particle);
	part_sys.init(init.center, init.size, init.speed, [5, 10], F, C);

	let grid_size = 75;
	let s = 1.0;
	let z = -.05;
	let sq = [0, 0, z, 0, s, z, s, s, z, s, s, z, 0, 0, z, s, 0, z];
	let sq_ind = 0;
	let grid = [];
	for(let x = -grid_size/2*s; x < grid_size/2*s; x += s){
		for(let y = -grid_size/2*s; y < grid_size/2*s; y += s, sq_ind++){
			let off = [x, y, 0];
			for(let i = 0; i < sq.length; i++){
				grid.push(sq[i] + off[i % 3]);
			}
		}
	}
	pos_buf = new Float32Array(part_sys.num*3 + part_sys.FC_data_num*3 + grid.length);
	col_buf = new Float32Array(part_sys.num*3 + part_sys.FC_data_num*3 + grid.length);
	for(let i = 0; i < grid.length; i++){
		pos_buf[(part_sys.num + part_sys.FC_data_num)*3 + i] = grid[i];
	}
	let color = [];
	for(let i = 0; i < part_sys.num*3; i++){
		if(i % 3 == 0){
			color = [.5, .5, .5];
			color[Math.floor(Math.random()*3)] = 1;
		}
		col_buf[i] = color[i % 3];
	}
	for(let i = 0; i < part_sys.FC_data_num*3; i++){
		col_buf[part_sys.num*3 + i] = 1;
	}
	for(let i = 0; i < grid.length; i++){
		col_buf[(part_sys.num + part_sys.FC_data_num)*3 + i] = .2*(Math.floor(i/sq.length) % 2);
	}

	fsize = pos_buf.BYTES_PER_ELEMENT;
	var gl_col_buf = gl.createBuffer();
	gl.bindBuffer(gl.ARRAY_BUFFER, gl_col_buf);
	gl.bufferData(gl.ARRAY_BUFFER, col_buf, gl.STATIC_DRAW);
	var a_Color = gl.getAttribLocation(gl.program, 'a_Color');
	gl.vertexAttribPointer(a_Color, 3, gl.FLOAT, false, 3*fsize, 0);
	gl.enableVertexAttribArray(a_Color);

	var gl_pos_buf = gl.createBuffer();
	gl.bindBuffer(gl.ARRAY_BUFFER, gl_pos_buf);
	gl.bufferData(gl.ARRAY_BUFFER, pos_buf, gl.DYNAMIC_DRAW);
	var a_Position = gl.getAttribLocation(gl.program, 'a_Position');
	gl.vertexAttribPointer(a_Position, 3, gl.FLOAT, false, 3*fsize, 0);
	gl.enableVertexAttribArray(a_Position);

	var model_matrix = new Matrix4();
	var view_matrix = new Matrix4();
	var proj_matrix = new Matrix4();
	view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
	proj_matrix.setPerspective(70, c.width/c.height, .01, 500);
	u_ModelMatrix = gl.getUniformLocation(gl.program, 'u_ModelMatrix');
	u_ViewMatrix = gl.getUniformLocation(gl.program, 'u_ViewMatrix');
	u_ProjMatrix = gl.getUniformLocation(gl.program, 'u_ProjMatrix');
	u_Point = gl.getUniformLocation(gl.program, 'u_Point');

	gl.uniformMatrix4fv(u_ModelMatrix, false, model_matrix.elements);
	gl.uniformMatrix4fv(u_ViewMatrix, false, view_matrix.elements);
	gl.uniformMatrix4fv(u_ProjMatrix, false, proj_matrix.elements);

	let last_t = Date.now();
	var tick = function(){
		let this_t = Date.now();
		let elapsed = this_t - last_t;
		last_t = this_t;

		if(!paused && elapsed < 500){
			part_sys.applyAllForces(part_sys.s1, part_sys.F);
			part_sys.s1dot = part_sys.dotFinder(part_sys.s1);
			part_sys.solver(elapsed);
			part_sys.doConstraint(part_sys.s1, part_sys.s2, part_sys.C);
			part_sys.render(gl, gl_pos_buf);
			part_sys.swap();
		}
		cam.strafe(elapsed);
		view_matrix.setLookAt(cam.pos[0], cam.pos[1], cam.pos[2], cam.foc[0], cam.foc[1], cam.foc[2], 0, 0, 1);
		gl.uniformMatrix4fv(u_ViewMatrix, false, view_matrix.elements);

		draw(gl);
		requestAnimationFrame(tick, c);
	}
	tick();

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

function draw(gl){
	gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

	gl.uniform1i(u_Point, 1);
	gl.drawArrays(gl.POINTS, 0, part_sys.num);

	gl.uniform1i(u_Point, 0);
	if(part_sys.FC_data_num > 0){
		gl.drawArrays(gl.LINES, part_sys.num, part_sys.FC_data_num);
	}
	gl.drawArrays(gl.TRIANGLES, part_sys.num + part_sys.FC_data_num, (pos_buf.length - (part_sys.num + part_sys.FC_data_num)*3)/3);
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
