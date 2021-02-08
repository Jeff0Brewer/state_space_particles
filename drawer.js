class Drawer{
	constructor(shader_inds, buffer_breaks, draw_types){
		this.sh = shader_inds;
		this.brk = buffer_breaks;
		this.typ = draw_types;
		this.fpv = FPV;

		let len = 0;
		for(let i = 0; i < this.brk.length; i++){
			len += this.brk[i];
		}
		let buffer = new Float32Array(len*this.fpv);
		this.fsize = buffer.BYTES_PER_ELEMENT;

		this.gl_buf = gl.createBuffer();
		gl.bindBuffer(gl.ARRAY_BUFFER, this.gl_buf);
		gl.bufferData(gl.ARRAY_BUFFER, buffer, gl.DYNAMIC_DRAW);


		for(let i = 0; i < this.sh.length; i++){
			switch_shader(this.sh[i]);
			let a_Position = gl.getAttribLocation(gl.program, 'a_Position');
			gl.vertexAttribPointer(a_Position, 3, gl.FLOAT, false, this.fsize*this.fpv, 0);
			gl.enableVertexAttribArray(a_Position);

			let a_Color = gl.getAttribLocation(gl.program, 'a_Color');
			gl.vertexAttribPointer(a_Color, 4, gl.FLOAT, false, this.fsize*this.fpv, 3*this.fsize);
			gl.enableVertexAttribArray(a_Color);

			let a_Size = gl.getAttribLocation(gl.program, 'a_Size');
			gl.vertexAttribPointer(a_Size, 1, gl.FLOAT, false, this.fsize*this.fpv, 7*this.fsize);
			gl.enableVertexAttribArray(a_Size);
		}
	}

	buffer_data = function(start_ind, data){
		gl.bindBuffer(gl.ARRAY_BUFFER, this.gl_buf);
		gl.bufferSubData(gl.ARRAY_BUFFER, start_ind, data);
	}

	draw(){
		gl.bindBuffer(gl.ARRAY_BUFFER, this.gl_buf);
		let offset = 0;
		let last_sh = -1;
		for(let i = 0; i < this.sh.length; i++){
			if(this.brk[i] > 0){
				if(this.sh[i] != last_sh){
					switch_shader(this.sh[i]);
					gl.vertexAttribPointer(gl.getAttribLocation(gl.program, 'a_Position'), 3, gl.FLOAT, false, this.fsize*this.fpv, 0);
					gl.vertexAttribPointer(gl.getAttribLocation(gl.program, 'a_Color'), 4, gl.FLOAT, false, this.fsize*this.fpv, 3*this.fsize);
					gl.vertexAttribPointer(gl.getAttribLocation(gl.program, 'a_Size'), 1, gl.FLOAT, false, this.fsize*this.fpv, 7*this.fsize);
					last_sh = this.sh[i];
				}
				gl.drawArrays(this.typ[i], offset, this.brk[i]);
				offset += this.brk[i];
			}
		}
	}
}