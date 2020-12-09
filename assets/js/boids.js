const canvas = document.getElementById('canvas');
console.log(canvas);
console.log(canvas.width);

const ctx = canvas.getContext('2d');
console.log(ctx)


function initSimulation () {
    console.log('Initializing simulation...');
}

/////////////////////////////////////////
// Boids
class Boid {
    constructor() {
	this.x = Math.random() * 500;
	this.y = Math.random() * 500;
	this.vel = 5;
    }

    // update() {
    // 	this.x += this.vel;
    // 	this.y += this.vel;
    // }
}


function getDistance(boid, other) {
    let dist = ((boid.x - other.x) ** 2) + ((boid.y - other.y) ** 2)
    return Math.sqrt(dist)
}

function findFlockmates(boid, flock) {
    let flockmates = flock.filter(other => {
	return getDistance(boid, other) <= boid.maxPerceptionRadius
    })
    return flockmates
}


////////////////////////////////////////
// Steering forces

function align(boid, other) {    
}

function separate(boid, other) {
}

function cohere(boid, other) {
}

function calcSteeringForce(boid) {
}

////////////////////////////
// Animation

function drawBoid(boid) {
    ctx.moveTo(boid.x, boid.y);
    ctx.arc(boid.x, boid.y, 5, 0, Math.PI * 2, false);
    ctx.fill();
    ctx.stroke();
    console.log('drew boid');
}






function runSim(flock) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    flock.forEach(boid => {
	drawBoid(boid);
	boid.x += boid.vel;
	boid.y += boid.vel;
    })
   
    requestAnimationFrame(runSim);
}

//////////////////////////////////
//// Test Run
initSimulation();

const flock = []
for (let i=0; i < 5; i++) {
    console.log(i);
    flock.push(new Boid());
}

console.log(flock)

console.table(flock);
flock.forEach(boid => drawBoid(boid));

// runSim(flock);


    
//for (boid of flock) {
//    drawBoid(boid);
//}
