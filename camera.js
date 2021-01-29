class CameraController{
	constructor(cam, focus, rot_speed, zoom_speed){
		this.dragging = false;
		this.mouse = {
			x: 0,
			y: 0
		};
		this.rotation = new Matrix4();
		this.pos = cam;
		this.r_spd = rot_speed;
		this.z_spd = -zoom_speed;
		this.foc = focus;
		this.strafe_sign = {
			lat: 0,
			for: 0
		};
	}

	strafe(elapsed){
		let speed = 10;
		let d = add(mult_scalar(norm(sub(this.foc.slice(0, 2), this.pos.slice(0, 2))).concat([0]), this.strafe_sign.for*speed*elapsed/1000), mult_scalar(norm(cross3(sub(this.foc, this.pos), [0, 0, 1])), this.strafe_sign.lat*speed*elapsed/1000));
		this.pos = add(this.pos, d);
		this.foc = add(this.foc, d);
	}

	add_strafe(s){
		this.strafe_sign.lat = Math.sign(this.strafe_sign.lat + s[0]);
		this.strafe_sign.for = Math.sign(this.strafe_sign.for + s[1]);
	}

	mousedown(e){
		this.dragging = true;
		this.mouse.x = e.clientX;
		this.mouse.y = e.clientY;
	}

	mousemove(e){
		if(this.dragging){
			let dx = this.r_spd * (e.clientX - this.mouse.x);
			let dy = this.r_spd * (e.clientY - this.mouse.y);
			
			let dir = sub(this.foc, this.pos);
			
			let ax = norm(cross3(dir, [0, 0, 1]));

			this.rotation = new Matrix4();
			this.rotation.rotate(-dx, 0, 0, 1);
			this.rotation.rotate(-dy, ax[0], ax[1], ax[2]);

			let f = new Vector3(dir);

			this.foc = add(this.pos, this.rotation.multiplyVector3(f).elements);

			this.mouse.x = e.clientX;
			this.mouse.y = e.clientY;
		}
	}

	mouseup(e){
		this.dragging = false;
	}

	wheel(e){
		let d = mult_scalar(sub(this.foc, this.pos), e.deltaY*this.z_spd);
		this.pos = add(this.pos, d);
		this.foc = add(this.foc, d);
	}
}
