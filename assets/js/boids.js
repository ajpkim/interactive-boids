const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const MAX_X = canvas.width;
const MAX_Y = canvas.height;

const alignRangeSlider = document.getElementById('alignRangeSlider')
const alignForceSlider = document.getElementById('alignForceSlider')

const separateRangeSlider = document.getElementById('separateRangeSlider')
const separateForceSlider = document.getElementById('separateForceSlider')

const cohereRangeSlider = document.getElementById('cohereRangeSlider')
const cohereForceSlider = document.getElementById('cohereForceSlider')

const predatorAvoidanceRangeSlider = document.getElementById('predatorAvoidanceRangeSlider')
const predatorAvoidanceForceSlider = document.getElementById('predatorAvoidanceForceSlider')

const predatorPerceptionRangeSlider = document.getElementById('predatorPerceptionRangeSlider')
const predatorCountSlider = document.getElementById('predatorCountSlider')
//////////////////////////////////////////////////
// Slider variables:
let alignRange;
let separateRange;
let cohereRange;
let predatorAvoidanceRange;

let alignMagnitude;
let separateMagnitude;
let cohereMagnitude;
let predatorAvoidanceMagnitude;
//////////////////////////////////////////////////
// Simulation variables
// Boids
let boidMinSpeed = 2;
let boidMaxSpeed = 3;
let boidRadius = 10;
let boidTailLength = 5;
let boidSideLength = 9;
let boidSideRadians = 2.5;
let boidColor = 'blue';

// Predators
let predatorMinSpeed = 1;
let predatorChaseSpeed = boidMaxSpeed * 0.9;
// let predatorRadius = 10;
let predatorTailLength = boidTailLength * 2.5;
let predatorSideLength = boidSideLength * 2;
//let predatorMaxAngleDelta = 5;
let predatorSideRadians = 2.75;
let predatorPerceptionRange = 75;
// let predatorSize = 15;
let predatorColor = 'red';

// General
const rangeMultiplier = 10;
const forceMultiplier = 0.04;
let edgeBuffer = 20;
let numBoids = 125;
let boidIdCount = 0;
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
    if (boid.position.x + edgeBuffer < 0) {
	boid.position.x = MAX_X;
    } else if (boid.position.x - edgeBuffer > MAX_X) {
	boid.position.x = 0;
    }
    if (boid.position.y + edgeBuffer < 0) {
	boid.position.y = MAX_Y
    } else if (boid.position.y - edgeBuffer > MAX_Y) {
	boid.position.y = 0;
    }
}

//////////////////////////////////////////////////
class Vector2d {
    constructor(x, y) {
	this.x = x;
	this.y = y;
    };
    
    copy() {
	return new Vector2d(this.x, this.y);
    }
    
    magnitude() {
	return Math.sqrt(this.x**2 + this.y**2);
    }
    
    // Compute unit vector in current direction and scale to desired magnitude.
    setMagnitude(mag) {
	if (this.magnitude() === 0) {
	    return this;
	} else {
	    this.div(this.magnitude());
	    this.mult(mag);
	    return this;
	}
    }

    // Force the magnitude to be smaller than global boidMaxSpeed (keep direction).
    limitMagnitude() {
	if (this.magnitude() > boidMaxSpeed) {
	    this.setMagnitude(boidMaxSpeed);
	}
	return this;
    }

    normalize() {
	return this.div(this.magnitude());
    }

    euclideanDistance(other) {
	return Math.sqrt((this.x - other.x)**2 + (this.y - other.y)**2);
    }
    
    // Basic vector and scalar arithmetic operations:
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
	    return this;
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
	if (other === 0) {
	    throw new Error(`Zero Division Error`);
	} else if (typeof other === "number") {
	    this.x /= other;
	    this.y /= other;
	    return this;
	} else {
	    throw new TypeError(`Invalid scalar division. ${other} is not a scalar`);
	}
    }
    
    dot(other) {
	if (other instanceof Vector2d) {
	    return (this.x * other.x) + (this.y * other.y);
	} else {
	    throw new TypeError(`Invalid dot product. ${other} is not a Vector2d instance`);
	}
    }

    // Calculate and return the angle between this vector and arg vector.
    angleBetween(other) {
	if (other instanceof Vector2d) {
	    let cosineTheta = this.dot(other) / (this.magnitude() + other.magnitude());
	    return Math.acos(cosineTheta);
	} else {
	    throw new TypeError(`Cannot compute angle between ${this} and arg: ${other}`);
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
	this.position = new Vector2d(randomNumber(0, MAX_X), randomNumber(0, MAX_Y));
	this.velocity = new Vector2d(randomNumber(-boidMaxSpeed, boidMaxSpeed), randomNumber(-boidMaxSpeed, boidMaxSpeed))
	this.velocity.setMagnitude(randomNumber(boidMinSpeed, boidMaxSpeed));
	this.acceleration = new Vector2d(0, 0);
	this.alignMates = [];
	this.separateMates = [];
	this.cohereMates = [];	
    }
   
    // Calculate and apply the alignment, cohesion, and separation forces to boid acceleration:
    
    // Velocity matching. Steer towards the average velocity (orientation) of local flockmates.
    align() {
	if (this.alignMates.length === 0) return;
	let alignForce = new Vector2d(0, 0);
	for (let other of this.alignMates) {
	    alignForce.add(other.velocity);
	}
	alignForce.div(this.alignMates.length);
	alignForce.setMagnitude(this.velocity.magnitude());
	alignForce.subtract(this.velocity);
	alignForce.setMagnitude(alignMagnitude);
	this.acceleration.add(alignForce);
    }
    // Collision avoidance. Steer to avoid crowding local flockmates.
    separate() {
	if (this.separateMates.length === 0) return;
	let separateForce = new Vector2d(0, 0);
	for (let other of this.separateMates) {
	    let distance = boidDistances[this.id][other.id];
	    let diffVector = this.position.copy().subtract(other.position);
	    
	    // Weight further away boids less.
	    diffVector.div(distance**2);
	    separateForce.add(diffVector);
	}
	//separateForce.div(this.separateMates.length);
	separateForce.setMagnitude(this.velocity.magnitude());
	separateForce.subtract(this.velocity);
	separateForce.setMagnitude(separateMagnitude);
	this.acceleration.add(separateForce);
    }
    // Flock centering. Steer towards the average position of local flockmates.
    cohere() {
	if (this.cohereMates.length === 0) return;
	let cohereForce = new Vector2d(0, 0);
	for (let other of this.cohereMates) {
	    cohereForce.add(other.position);
	};
	cohereForce.div(this.cohereMates.length);
	// Get vector pointing towards average location by subtracting current location.
	cohereForce.subtract(this.position);
	cohereForce.subtract(this.velocity);
	cohereForce.setMagnitude(cohereMagnitude);
	this.acceleration.add(cohereForce);
    }

    avoidPredators() {
	let predatorAvoidanceForce = new Vector2d(0,0);
	for (let predator of predators) {
	    let distance = this.position.euclideanDistance(predator.position);
	    if (distance <= predatorAvoidanceRange) {
		let diffVector = this.position.copy().subtract(predator.position);
		diffVector.div(distance ** 2);
		predatorAvoidanceForce.add(diffVector);
	    }
	}

	/// SOMETHING IS WRONG HERE!!!
	if (predatorAvoidanceForce.magnitude() > 0) {
	    predatorAvoidanceForce.setMagnitude(predatorAvoidanceMagnitude);
	}
	
	this.acceleration.add(predatorAvoidanceForce);
    }

    // Calculate the align, separate, and cohere forces and apply them to acceleration.
    applySteeringForces() {
	this.align();
	this.cohere();
	this.separate();
	this.avoidPredators();
    }
    
    // Move boid position according to velocity after updating velocity
    // by adding acceleration (wrap edges). Set acceleration to 0 afterwards.
    updatePosition() {
	this.applySteeringForces()
	this.velocity.add(this.acceleration);
	// Ensure boidMinSpeed <= boid speed i.e. boid.velocity.magnitude() <= boidMaxSpeed
	this.velocity.setMagnitude(Math.max(this.velocity.magnitude(), boidMinSpeed));
	this.velocity.limitMagnitude(boidMaxSpeed);
	this.position.add(this.velocity);
	wrapEdges(this);
	this.acceleration.mult(0);
    }
}

class Predator {
    constructor () {
	this.position = new Vector2d(randomNumber(0, MAX_X), randomNumber(0, MAX_Y));
	this.velocity = new Vector2d(randomNumber(-predatorChaseSpeed, predatorChaseSpeed), randomNumber(-predatorChaseSpeed, predatorChaseSpeed))
	this.velocity.setMagnitude(randomNumber(predatorMinSpeed, predatorChaseSpeed));
	this.prey = null;
	this.acceleration = new Vector2d(0, 0);
    }

    // Loop through flock until find a boid within perception range, lock that boid as "prey"
    findPrey() {
	let prey;
	for (let boid of flock) {
	    if (this.position.euclideanDistance(boid.position) <= predatorPerceptionRange) {
		this.prey = boid;
		return
	    }
	}
	this.prey = null;  
    }
    
    chasePrey() {
	if ((this.prey === null) || (this.position.euclideanDistance(this.prey.position) > predatorPerceptionRange)) {
	    this.findPrey();
	}
	if (this.prey === null) {
	    // Apply 10% braking force acceleration
	    let brakingForce = this.velocity.copy().mult(-1);
	    brakingForce.setMagnitude(this.velocity.magnitude() * 0.1);
	    this.acceleration.add(brakingForce);
	    return
	}
		
	// let chaseForce = this.position.copy().subtract(this.prey.position);
	let chaseForce = this.prey.position.copy().subtract(this.position);
	chaseForce.subtract(this.velocity);
	chaseForce.setMagnitude(predatorChaseSpeed);
	
	// // Set a max delta
	// if (chaseForce.angleBetween(this.velocity) > predatorMaxAngleDelta) {
	//     console.log('WOAH');
	// }

	
	
	
	this.acceleration.add(chaseForce);
    }

    updatePosition() {
	this.chasePrey();
	this.velocity.add(this.acceleration);
	this.velocity.setMagnitude(Math.max(predatorMinSpeed, this.velocity.magnitude()));
	this.velocity.limitMagnitude(predatorChaseSpeed);
	this.position.add(this.velocity);
	this.acceleration.mult(0);
	wrapEdges(this);
	
    }
}

//////////////////////////////////////////////////
// Calculate and store the distance between each boid in a
// global 2d matrix and update each boids flockmates array.
function calcDistancesFlockmates() { 
   for (let [i, boid] of flock.entries()) {
	boid.alignMates.length = 0;
	boid.separateMates.length = 0;
	boid.cohereMates.length = 0;
	// Only iterate through each pair one time.
	for (let j = i + 1; j < flock.length; j++) {
	    other = flock[j]
	    let distance = boid.position.euclideanDistance(other.position);
	    // Update global distance matrix.
	    boidDistances[boid.id][other.id] = distance;
	    boidDistances[other.id][boid.id] = distance;
	    // Populate the flockmates for each force.
	    if (distance <= alignRange) {
		boid.alignMates.push(other);
		other.alignMates.push(boid);
	    }
	    if (distance <= separateRange) {
		boid.separateMates.push(other);
		other.separateMates.push(boid);
	    }
	    if (distance <= cohereRange) {
		boid.cohereMates.push(other);
		other.cohereMates.push(boid);
	    }
	}
    }
}    
//////////////////////////////////////////////////
// Animation

// There are 4 points. Order matters because of drawing sequence.
function getBoidDrawingCoords(boid) {
    points = [];
    let directionVector = boid.velocity.copy().normalize();    
    let directionRadians = Math.atan2(directionVector.y, directionVector.x);

    let side1Radians = directionRadians + boidSideRadians;
    let side1 = new Vector2d(Math.cos(side1Radians), Math.sin(side1Radians));
    side1.setMagnitude(boidSideLength);
    side1 = boid.position.copy().add(side1);
    points.push([side1.x, side1.y]);

    let tailVector = directionVector.mult(-1).setMagnitude(boidTailLength);
    let tailPoint = boid.position.copy().add(tailVector);
    points.push([tailPoint.x, tailPoint.y]);

    let side2Radians = directionRadians - boidSideRadians;
    let side2 = new Vector2d(Math.cos(side2Radians), Math.sin(side2Radians));
    side2.setMagnitude(boidSideLength);
    side2 = boid.position.copy().add(side2);
    points.push([side2.x, side2.y]);

    // Add boid head position as final coordinate for drawing loop.
    points.push([boid.position.x, boid.position.y]);

    return points
}

function drawBoid(boid) {
    coordsArr = getBoidDrawingCoords(boid);
    ctx.moveTo(boid.position.x, boid.position.y);
    // for circle boids
    // ctx.arc(boid.position.x, boid.position.y, boidRadius, 0, Math.PI * 2, false);    
    ctx.beginPath();
    for (coords of coordsArr) ctx.lineTo(coords[0], coords[1]);
    ctx.fillStyle = boidColor;
    ctx.fill();
}


function getPredatorDrawingCoords(predator) {
    points = [];
    let directionVector = predator.velocity.copy().normalize();    
    let directionRadians = Math.atan2(directionVector.y, directionVector.x);

    let side1Radians = directionRadians + predatorSideRadians;
    let side1 = new Vector2d(Math.cos(side1Radians), Math.sin(side1Radians));
    side1.setMagnitude(predatorSideLength);
    side1 = predator.position.copy().add(side1);
    points.push([side1.x, side1.y]);

    let tailVector = directionVector.mult(-1).setMagnitude(predatorTailLength);
    let tailPoint = predator.position.copy().add(tailVector);
    points.push([tailPoint.x, tailPoint.y]);

    let side2Radians = directionRadians - predatorSideRadians;
    let side2 = new Vector2d(Math.cos(side2Radians), Math.sin(side2Radians));
    side2.setMagnitude(predatorSideLength);
    side2 = predator.position.copy().add(side2);
    points.push([side2.x, side2.y]);

    // Add predator head position as final coordinate for drawing loop.
    points.push([predator.position.x, predator.position.y]);

    return points
}

function drawPredator(predator) {
    coordsArr = getPredatorDrawingCoords(predator);
    ctx.moveTo(predator.position.x, predator.position.y);
    // for circle predators
    // ctx.arc(predator.position.x, predator.position.y, predatorRadius, 0, Math.PI * 2, false);    
    ctx.beginPath();
    for (coords of coordsArr) ctx.lineTo(coords[0], coords[1]);
    ctx.fillStyle = predatorColor;
    ctx.fill();
}

//////////////////////////////////////////////////

function updateSliders() {
    // Primary flocking interactions
    alignRange = alignRangeSlider.value * rangeMultiplier;
    separateRange = separateRangeSlider.value * rangeMultiplier;
    cohereRange = cohereRangeSlider.value * rangeMultiplier;
    
    alignMagnitude = alignForceSlider.value * forceMultiplier;
    separateMagnitude = separateForceSlider.value * forceMultiplier;
    cohereMagnitude = cohereForceSlider.value * forceMultiplier;

    // Predator interactions
    predatorAvoidanceRange = predatorAvoidanceRangeSlider.value * rangeMultiplier;
    predatorAvoidanceMagnitude = predatorAvoidanceForceSlider.value * (forceMultiplier * 2) // Arb *2

    predatorPerceptionRange = predatorPerceptionRangeSlider.value * (rangeMultiplier * 0.6); // Arbitrary predator range discount
    numPredators = predatorCountSlider.value;
}

function updatePredators() {
    while (predators.length < numPredators) {
	predators.push(new Predator());
    }
    while (predators.length > numPredators) {
	predators.pop();
    }
    predators.forEach(predator => {
	predator.updatePosition();
	drawPredator(predator);
    });
}




function runSim() {
    updateSliders();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    calcDistancesFlockmates();
    flock.forEach((boid) => {
	drawBoid(boid);	
	boid.updatePosition();
    });

    updatePredators();
    requestAnimationFrame(runSim);
}

// For testing.
function step() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    updateSliders();
    calcDistancesFlockmates();
    flock.forEach((boid) => {
	drawBoid(boid);	
	boid.updatePosition();
    });
}

//////////////////////////////////////////////////
// Initialize and run simulation.
initSimulation();

const flock = []
for (let i=0; i < numBoids; i++) {
    flock.push(new Boid());
}

flock.forEach(boid => drawBoid(boid));

let predators = [];

 // Create 2d array to hold boid distances i.e. [0][2] holds dist from boid 0 to boid 2
let boidDistances = Array.from(Array(flock.length), () => new Array(flock.length))

// pred = new Predator();
// drawPredator(pred);

runSim(flock);
