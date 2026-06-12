
#include "boid.h"
#include "obstacles.h"
#include "workerpool.h"
#include "uniformgrid.h"
#include "poi.h"

void printStats(void);

vec2 GetCameraInputDirection();
void DrawBorders();

void DrawBoidsBatchedPoints(Camera* camera, Boid* boids, int numBoids, SimulationParameters* sim, bool rebuildPoints);

void HandleObstacleInput(Obstacles** obstacles);

void RunFlockWorkers(WorkerPool* workerpool, UniformGrid* grid, int boidCount, Boid** boids, Boid** boidsNext, Obstacles* obstacles, SimulationParameters* sim, PointOfInterest* pointsOfInterest, int poiCount, real deltaTime);


void DrawBoidsBatchedPointsInterpolated(
	Camera* camera,
	const Boid* previous,
	const Boid* current,
	int numBoids,
	SimulationParameters* sim,
	real alpha,
	bool rebuildPoints);


int main(int argc, char *argv[]);
