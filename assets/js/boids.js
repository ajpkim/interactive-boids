const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

const MAX_X = canvas.width;
const MAX_Y = canvas.height;

let maxPerceptionRange = 75;
let speedLimit = 3;
let boidIdCount = 0;

// Slider variables:
let alignMagnitude = 0;
let separateMagnitude = 100;
let cohereMagnitude = 0;

let alignRange = 75;
let separateRange = 25;
let cohereRange = 75;

//////////////////////////////////////////////////
function initSimulation () {
    console.log('Initializing simulation...');
}

//////////////////////////////////////////////////
// Helper functions:

// Return a random number within range [min, max].
function randomNumber(min, max) {
    return Math.random() * (max - min) + min;
}

// Make boid position wrap around canvas instead of fly continually off-screen.
function wrapEdges(boid) {
    if (boid.position.x + boid.size < 0) {
	boid.position.x = MAX_X;
    } else if (boid.position.x - boid.size > MAX_X) {
	boid.position.x = 0;
    }
    if (boid.position.y + boid.size < 0) {
	boid.position.y = MAX_Y
    } else if (boid.position.y - boid.size > MAX_Y) {
	boid.position.y = 0;
    }
}

//////////////////////////////////////////////////
class Vector2d {
    constructor(x, y) {
	this.x = x;
	this.y = y;
    }
    copy() { return new Vector2d(this.x, this.y) };
    magnitude() { return Math.sqrt(this.x**2 + this.y**2) };
    setMagnitude(mag) {
	if (this.magnitude() === 0) {
	    return this;
	} else {
	    this.div(this.magnitude());
	    this.mult(mag);
	    return this;
	}
    }
    limitMagnitude() {
	if (this.magnitude() > speedLimit) {
	    this.setMagnitude(speedLimit)
	}
	return this;
    }
    
    normalize() { return this.setMagnitude(1) };

    euclideanDistance(other) {
	return Math.sqrt((this.x - other.x)**2 + (this.y - other.y)**2);
    }
    
// Basic vector and scalar arithmetic operations.
    add(other) {
	if (other instanceof Vector2d) {
	    this.x += other.x;
	    this.y += other.y;
	    return this;
	} else if (typeof other === "number") {
	    this.x += other;
	    this.y += other;
	    return this;
	} else {
	    throw new TypeError(`Invalid addition. ${other} is neither vector nor scalar`);
	}
    }
    subtract(other) {
	if (other instanceof Vector2d) {
	    this.x -= other.x;
	    this.y -= other.y;
	    return this;
	} else if (typeof other === "number") {
	    this.x -= other;
	    this.y -= other;
	    return true;
	} else {
	    throw new TypeError(`Invalid subtraction. ${other} is neither vector nor scalar`);
	}
    }
    // Scalar multipication.
    mult(other) {
	if (typeof other === "number") {
	    this.x *= other;
	    this.y *= other;
	    return this;
	} else {
	    throw new TypeError(`Invalid scalar multiplication. ${other} is not a scalar`);
	}
    }
    // Scalar division.
    div(other) {
	if (typeof other === "number") {
	    this.x /= other;
	    this.y /= other;
	    return this;
	} else {
	    throw new TypeError(`Invalid scalar division. ${other} is not a scalar`);
	}
    }
    
    toString() {
	return `<Vector2d obj with position (${this.x, this.y})>`;
    }
}
//////////////////////////////////////////////////
class Boid {
    constructor() {
	this.id = boidIdCount++;
	this.size = 5
	this.position = new Vector2d(randomNumber(0, MAX_X), randomNumber(0, MAX_Y));
	this.velocity = new Vector2d(randomNumber(-speedLimit, speedLimit), randomNumber(-speedLimit, speedLimit)).limitMagnitude(speedLimit);
	this.acceleration = new Vector2d(0, 0);
	this.flockmates = new Set()
    }
   
    getForceFlockmates() {
	let forceFlockmates = {
	    alignMates: [],
	    cohereMates: [],
	    separateMates: [],
	}
	for (let flockmate of this.flockmates) {
	    let distance = boidDistances[this.id][flockmate.id];
	    if (distance <= alignRange) forceFlockmates["alignMates"].push(flockmate);
	    if (distance <= cohereRange) forceFlockmates["cohereMates"].push(flockmate);
	    if (distance <= separateRange) forceFlockmates["separateMates"].push(flockmate);
	}
	return forceFlockmates;
    }

    // Calculate and apply the alignment, cohesion, and separation forcee to boid acceleration:
    
    // Flock centering.
    align(alignMates) {
	if (alignMates.length === 0) return;
	let alignForce = new Vector2d(0, 0);
	for (let other of alignMates) {
	    alignForce.add(other.velocity);
	};
	alignForce.div(alignMates.length);
	alignForce.setMagnitude(alignMagnitude);
	this.acceleration.add(alignForce);
    }
    // Collision avoidance.
    separate(separateMates) {
	if (separateMates.length === 0) return;
	let separateForce = new Vector2d(0, 0);
	for (let other of separateMates) {
	    let distance = boidDistances[this.id][other.id];
	    let diffVector = this.position.copy().subtract(other.position);
	    
	    // Weight further away boids less for separation purposes.
	    diffVector.div(distance);
	    separateForce.add(diffVector.mult(-1));
	}
	separateForce.div(separateMates.length);
	separateForce.setMagnitude(separateMagnitude);
	this.acceleration.add(separateForce);
    }
    // Flock centering.
    cohere(cohereMates) {
	if (cohereMates.length === 0) return;
	let cohereForce = new Vector2d(0, 0);
	for (let other of cohereMates) {
	    cohereForce.add(other.position);
	};
	cohereForce.div(cohereMates.length);
	cohereForce.setMagnitude(cohereMagnitude);
	this.acceleration.add(cohereForce);
    }
    
    calcSteeringForces() {
	let { alignMates, cohereMates, separateMates } = this.getForceFlockmates();
	this.align(alignMates);
	this.cohere(cohereMates);
	this.separate(separateMates);
    }

    updatePosition() {
	this.calcSteeringForces();
	this.velocity.add(this.acceleration)
	this.velocity.limitMagnitude(speedLimit);
	this.position.add(this.velocity);
	this.acceleration.mult(0);
	wrapEdges(this);
    }
}

//////////////////////////////////////////////////
// Calculate and store the distance between each boid in a global 2d matrix and update each boids flockmates array.
function calcDistancesFlockmates() {
    for (let [i, boid] of flock.entries()) {
	boid.flockmates.clear();
	// Only iterate through each pair one time.
	for (let j = i + 1; j < flock.length; j++) {
	    other = flock[j]
	    let distance = boid.position.euclideanDistance(other.position);
	    // Update global distance matrix.
	    boidDistances[boid.id][other.id] = distance;
	    boidDistances[other.id][boid.id] = distance;
	    // Update flockmates for both boids.
	    if (distance <= maxPerceptionRange) {
		boid.flockmates.add(other);
		other.flockmates.add(boid);
	    }
	}
    }
}    
//////////////////////////////////////////////////
// Animation
function drawBoid(boid) {
    ctx.moveTo(boid.position.x, boid.position.y);
    ctx.arc(boid.position.x, boid.position.y, boid.size, 0, Math.PI * 2, false);
    ctx.fill();
    ctx.stroke();  // What does this do? Doesn't appear necessary...
}

function runSim() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    flock.forEach((boid) => {
	drawBoid(boid);	
	boid.updatePosition();
    });    
    requestAnimationFrame(runSim);
}

//////////////////////////////////////////////////
// Test Run
initSimulation();

const flock = []
for (let i=0; i < 20; i++) {
    flock.push(new Boid());
}

// flock.forEach(boid => drawBoid(boid));

let boidDistances = Array.from(Array(flock.length), () => new Array(flock.length))  // creates 2d array to hold boid distances i.e. [0][2] holds dist from boid 0 to boid 2

calcDistancesFlockmates();
runSim(flock);
