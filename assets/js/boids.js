const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');


const MAX_X = canvas.width;
const MAX_Y = canvas.height;
let speedLimit = 5;

////////////////////////////////


function initSimulation () {
    console.log('Initializing simulation...');
}

////////////////////////////////////////
// Helper functions
function randomNumber(min, max) {
    return Math.random() * (max - min) + min;
}

function getDistance(boid, other) {
    const dist = ((boid.x - other.x) ** 2) + ((boid.y - other.y) ** 2)
    return Math.sqrt(dist)
}



/////////////////////////////////////////
// Boids
class Boid {
    constructor() {
	this.id = boidIdCount++;
	this.size = 5
	this.x = randomNumber(0, MAX_X);
	this.y = randomNumber(0, MAX_Y);
	this.velocity = randomNumber(0, speedLimit);
	this.acceleration = 0
	this.flockmates = new Set()
    }

    // Make boids wrap canvas if position is off canvas
    set x(xCoord) {
	if (xCoord < 0) {
	    this._x = MAX_X;
	} else if (xCoord > MAX_X) {
	    this._x = 0;
	} else {
	    this._x = xCoord;
	}
    }
    get x() {
	return this._x;
    }
    set y(yCoord) {
	if (yCoord < 0) {
	    this._y = MAX_Y;
	} else if (yCoord > MAX_Y) {
	    this._y = 0;
	} else {
	    this._y = yCoord;
	}
    }
     get y() {
	return this._y
    }
}


function calcDistancesFlockmates() {
    for (let [i, boid] of flock.entries()) {
	boid.flockmates.clear();
	// Q: is slicing here bad bc of copy creations??
	// for (let other of flock.slice(i)) {
	for (let j = i + 1; j < flock.length; j++) {
	    other = flock[j]
	    let dist = getDistance(boid, other);
	    // Update global distance matrix
	    boidDistances[boid.id][other.id] = dist;
	    boidDistances[other.id][boid.id] = dist;
	    // Update boid flockmates
	    if (dist <= maxPerceptionRange) {
		boid.flockmates.add(other);
		other.flockmates.add(boid);
	    }
	}
    }
}



// function findFlockmates(boid, flock) {
//     let flockmates = flock.filter(other => {
// 	return getDistance(boid, other) <= boid.maxPerceptionRadius
//     })
//     return flockmates
// }


////////////////////////////////////////
// Steering forces

// Is it going to be inefficient to filter map for each force??

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
    ctx.arc(boid.x, boid.y, boid.size, 0, Math.PI * 2, false);
    ctx.fill();
//    ctx.stroke();  // What does this do? Doesn't appear necessary...
}


function runSim() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    flock.forEach((boid) => {
	drawBoid(boid);
	boid.x += boid.velocity;
	boid.y += boid.velocity;
    })    
    requestAnimationFrame(runSim);
}

//////////////////////////////////
//// Test Run

let maxPerceptionRange = 100;
let boidIdCount = 0  // Class variables seem not standard in JS...??

initSimulation();

const flock = []
for (let i=0; i < 10; i++) {
    flock.push(new Boid());
}

flock.forEach(boid => drawBoid(boid));


let boidDistances = Array.from(Array(flock.length), () => new Array(flock.length))  // creates 2d array to hold boid distances i.e. [0][2] holds dist from boid 0 to boid 2

calcDistancesFlockmates();


runSim(flock);
