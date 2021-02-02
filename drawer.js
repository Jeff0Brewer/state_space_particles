class Drawer{
	constructor(shader_inds, buffer_breaks, draw_types){
		this.sh = shader_inds;
		this.brk = buffer_breaks;
		this.typ = draw_types;
		this.fpv = 7; // px, py, pz, cr, cg, cb, ca

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
		}
	}

	buffer_data = function(start_ind, data){
		gl.bindBuffer(gl.ARRAY_BUFFER, this.gl_buf);
		gl.bufferSubData(gl.ARRAY_BUFFER, start_ind, data);
	}

	draw(){
		gl.bindBuffer(gl.ARRAY_BUFFER, this.gl_buf);
		let offset = 0;
		for(let i = 0; i < this.sh.length; i++){
			if(this.brk[i] > 0){
				switch_shader(this.sh[i]);
				gl.vertexAttribPointer(gl.getAttribLocation(gl.program, 'a_Position'), 3, gl.FLOAT, false, this.fsize*this.fpv, 0);
				gl.vertexAttribPointer(gl.getAttribLocation(gl.program, 'a_Color'), 4, gl.FLOAT, false, this.fsize*this.fpv, 3*this.fsize);
				gl.drawArrays(this.typ[i], offset, this.brk[i]);
				offset += this.brk[i];
			}
		}
	}
}