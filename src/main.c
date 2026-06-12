#include "common.h"

#include "draw.h"
#include "init.h"
#include "input.h"
#include "main.h"
#include "flockbehavior.h"
#include "camera.h"
#include "stats.h"


App    app;
Stats stats;
Camera camera;

int main(int argc, char* argv[])
{
	memset(&app, 0, sizeof(App));
	srand(RNG_SEED);
	initSDL();

	//atexit(cleanup);
	atexit(printStats);
	real posX = 100.0;
	real posY = 100.0;

	camera.x = 0;
	camera.y = 0;
	camera.zoom = 1.0;
	camera.minZoom = 0.01f;
	camera.maxZoom = 50.0f;
	camera.screenW = SCREEN_WIDTH;
	camera.screenH = SCREEN_HEIGHT;

	SimulationParameters sim = {

		.topSpeed = R(TOP_SPEED),
		.minSpeed = R(MIN_SPEED),
		.turnSpeed = R(TURN_SPEED),
		.acceleration = R(ACCELERATION),

		.scale = R(BOID_SCALE),

		.avoidFactor = R(AVOID_FACTOR),
		.matchingFactor = R(MATCHING_FACTOR),
		.centeringFactor = R(CENTERING_FACTOR),
		.borderingFactor = R(BORDERING_FACTOR),
		.obstacleAvoidFactor = R(OBSTACLE_AVOID_FACTOR),

		.maxVisible = MAX_VISIBLE,
		.visionRadius = R(VISION_RADIUS),
		.visionRadiusSq = R(VISION_RADIUS) * R(VISION_RADIUS),
		.protectedRange = R(PROTECTED_RANGE),
		.protectedRangeSq = R(PROTECTED_RANGE) * R(PROTECTED_RANGE),

		.poiFactor = R(POI_FACTOR),

		.obstacleAvoidDistance = R(OBSTACLE_AVOID_DISTANCE),

		.texture = loadTexture(BOID_TEXTURE),
		.boidColor = (DrawColor){255, 0, 0, 255}
	};

	int boidCount = BOID_COUNT;

	Boid* boids = malloc(sizeof(Boid) * boidCount);

	Boid* boidsNext = malloc(sizeof(Boid) * boidCount);

	Boid* boidsInterpolate = malloc(sizeof(Boid) * boidCount);

	if (boids == NULL || boidsNext == NULL || boidsInterpolate == NULL)
	{
		SDL_Log("Failed to allocate boids");
		exit(1);
	}

	for (size_t i = 0; i < boidCount; i++)
	{
		boids[i] = boid_create(&sim);
	}


	UniformGrid grid;

	if (!UniformGrid_Init(&grid, R(WORLD_WIDTH), R(WORLD_HEIGHT), sim.visionRadius * R(2.0), boidCount)) {
		SDL_Log("Failed to initialize uniform grid");
		exit(1);
	}

	Obstacles* obstacles = NULL;

	Obstacles* nextObstacle = obstacles;

	int poiCount = NUM_POI;
	PointOfInterest* pointsOfInterest = malloc(sizeof(PointOfInterest) * poiCount);

	if (pointsOfInterest == NULL)
	{
		SDL_Log("Failed to allocate points of interest");
		exit(1);
	}
	for (size_t i = 0; i < poiCount; i++)
	{
		pointsOfInterest[i] = poi_create_random();
	}

	Uint64 lastCounter = SDL_GetPerformanceCounter();

	double fpsTimer = 0.0;
	Uint64 frameCount = 0;


	double flockTimeAccum = 0.0;
	int flockCallCount = 0;
	double statsTimer = 0.0;

	DrawColor poiColor = { 0, 255, 0, 255 };

	int numThreads = SDL_GetCPUCount();
	int countThreads = 3;
	SDL_Thread** threads = malloc(sizeof(SDL_Thread*) * numThreads);
	char (*threadNames)[32] = malloc(sizeof(*threadNames) * numThreads);

	for (int i = 0; i < numThreads; i++)
	{
		snprintf(threadNames[i], sizeof(threadNames[i]),
			"BoidWorker %d", i);
	}


	// init workerpool
	workerpool.numThreads = numThreads;
	workerpool.quit = 0;
	workerpool.generation = 0;
	workerpool.completed = 0;
	workerpool.mutex = SDL_CreateMutex();
	workerpool.chunkMutex = SDL_CreateMutex();
	workerpool.startCond = SDL_CreateCond();
	workerpool.doneCond = SDL_CreateCond();

	int* threadIds = malloc(sizeof(int) * numThreads);
	if (threadIds == NULL)
	{
		SDL_Log("Failed to allocate threadIds");
		exit(1);
	}

	for (int i = 0; i < numThreads; i++)
	{
		threadIds[i] = i;
		threads[i] = SDL_CreateThread(
			PersistentWorkerMainBalanced,
			threadNames[i],
			&threadIds[i]
		);
	}


	GridCountJobData gridCountJobData = {
		.grid = &grid,
		.boids = boids,
		.cellCount = grid.cellCount
	};

	gridCountJobData.localCounts =
		calloc(countThreads * grid.cellCount, sizeof(int));

	gridCountJobData.writeOffsets =
		malloc(sizeof(int) * countThreads * grid.cellCount);

	gridCountJobData.touchedCells =
		malloc(sizeof(int) * countThreads * boidCount);

	gridCountJobData.touchedCounts =
		malloc(sizeof(int) * countThreads);



	gridCountJobData.cellCount = grid.cellCount;
	gridCountJobData.maxTouchedPerThread = boidCount;

	if (gridCountJobData.localCounts == NULL || gridCountJobData.writeOffsets == NULL || gridCountJobData.touchedCells == NULL || gridCountJobData.touchedCounts == NULL) {
		SDL_Log("Failed to allocate local counts for grid job data");
		exit(1);
	}

	// Statistics
	Stats_Init(&stats);
	Stats titleStats;
	Stats_Init(&titleStats);

	FrameProfiler profiler = { 0 };

	static real simAccum = 0.0f;

	double fixedDt = 1.0 / 15.0;
	while (1)
	{
		Uint64 frameStart = SDL_GetPerformanceCounter();
		Profiler_Begin(&profiler, STAT_FRAME_WORK);
		double deltaTime =
			(double)(frameStart - lastCounter) /
			SDL_GetPerformanceFrequency();

		real deltaTimeReal = (real)deltaTime;

		lastCounter = frameStart;

		prepareScene();
		doInput();
		double fps = 1.0 / deltaTime;

		Stats_AddSample(&stats, STAT_FPS, fps);
		Stats_AddSample(&titleStats, STAT_FPS, fps);
		Profiler_Begin(&profiler, STAT_UPDATE);

		if (app.mouseWheelY != 0)
		{
			real zoomStep = 1.01f;
			real zoomFactor = powf(zoomStep, app.mouseWheelY);

			Camera_ZoomAt(
				&camera,
				(vec2) {
				app.mouseX, app.mouseY
			},
				zoomFactor
			);
		}

		if (frameCount % 16 == 0)
		{
			// prepare grid
			Profiler_Begin(&profiler, STAT_GRID_PREPARE);
			if (!UniformGrid_PrepareBuild(&grid, boidCount))
			{
				return 0;
			}
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_PREPARE);


			//gridMemsetStart = SDL_GetPerformanceCounter();
			Profiler_Begin(&profiler, STAT_GRID_WORK);

			// No more clearing numThreads * cellCount.
			// Only reset one int per thread.
			Profiler_Begin(&profiler, STAT_GRID_MEMSET);
			memset(gridCountJobData.touchedCounts,0,sizeof(int) * countThreads);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_MEMSET);

			gridCountJobData.boids = boids;

			int partitionSize = (boidCount + countThreads - 1) / countThreads;

			gridCountJobData.partitionSize = partitionSize;
			gridCountJobData.partitionCount = countThreads;
			
			Profiler_Begin(&profiler, STAT_GRID_COUNT);
			WorkerPool_Run(
				&workerpool,
				boidCount,
				partitionSize,
				countThreads,
				UniformGrid_RunGridCountJob,
				&gridCountJobData
			);
			//UniformGrid_Count(&grid, boids, boidCount);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_COUNT);

			
			// Reduce
			Profiler_Begin(&profiler, STAT_GRID_REDUCE);
			UniformGrid_GridCountReduce(&grid, &gridCountJobData, countThreads);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_REDUCE);

			// prefixsum
			Profiler_Begin(&profiler, STAT_GRID_PREFIX);
			UniformGrid_PrefixSum(&grid);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_PREFIX);

			Profiler_Begin(&profiler, STAT_GRID_OFFSET);
			UniformGrid_ComputeWriteOffsets(&grid, &gridCountJobData, countThreads);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_OFFSET);

			// build grid
			Profiler_Begin(&profiler, STAT_GRID_BUILD);
			WorkerPool_Run(
				&workerpool,
				boidCount,
				partitionSize,
				countThreads,
				UniformGrid_RunBuildJob,
				&gridCountJobData
			);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_BUILD);

			Profiler_Begin(&profiler, STAT_GRID_BUILD);
			UniformGrid_ClearLocalCounts(&gridCountJobData, countThreads);
			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_BUILD);

			Profiler_End(&profiler, &stats, &titleStats, STAT_GRID_WORK);
		}


		simAccum += deltaTimeReal;
		if (frameCount % 4 == 0) {

			memcpy(boidsInterpolate, boids, sizeof(Boid) * boidCount);

			Profiler_Begin(&profiler, STAT_FLOCK);
			RunFlockWorkers(
				&workerpool,
				&grid,
				boidCount,
				&boids,
				&boidsNext,
				obstacles,
				&sim,
				pointsOfInterest,
				poiCount,
				simAccum
			);
			Profiler_End(&profiler, &stats, &titleStats, STAT_FLOCK);
			simAccum = 0.0f;
		}
		
		

		for (size_t i = 0; i < poiCount; i++)
		{
			PointOfInterest* poi = &pointsOfInterest[i];

			GridQuery query = UniformGrid_GetQueryRange(
				&grid,
				poi->x,
				poi->y,
				poi->radius
			);

			for (int row = query.minRow; row <= query.maxRow; row++)
			{
				for (int col = query.minCol; col <= query.maxCol; col++)
				{
					const GridCell* cell = UniformGrid_GetCellConst(&grid, col, row);

					if (cell == NULL)
						continue;

					consume_poi(poi, cell->count);

				}
			}

		}


		Profiler_End(&profiler, &stats, &titleStats, STAT_UPDATE);


		Profiler_Begin(&profiler, STAT_DRAW);

		vec2 cameraInputDir = GetCameraInputDirection();
		camera.x += cameraInputDir.x * CAMERA_SPEED * R(deltaTime);
		camera.y += cameraInputDir.y * CAMERA_SPEED * R(deltaTime);

		//DrawBoids(boids, boidCount, &sim);
		bool rebuildDrawPoints = true; //frameCount % 2 == 0;
		//DrawBoidsBatchedPoints(&camera, boids, boidCount, &sim, rebuildDrawPoints);

		real alpha = simAccum / fixedDt;

		DrawBoidsBatchedPointsInterpolated(
			&camera,
			boidsInterpolate,
			boids,
			boidCount,
			&sim,
			alpha,
			rebuildDrawPoints
		);
		for (size_t i = 0; i < poiCount; i++)
		{
			if (!pointsOfInterest[i].active)
			{
				pointsOfInterest[i] = poi_reinitialize(&pointsOfInterest[i]);
			}

			//poi_draw(&camera, &pointsOfInterest[i], poiColor);
		}

		vec2 screenPos = WorldToScreen(&camera, (vec2) { boids[0].x, boids[0].y });

		/*draw_circle(screenPos.x, screenPos.y, sim.visionRadius * camera.zoom, (DrawColor) { 255, 0, 255, 255 }, false);

		draw_circle(screenPos.x, screenPos.y, sim.protectedRange * camera.zoom, (DrawColor) {255, 0, 255, 255}, false);*/

		HandleObstacleInput(&obstacles);


		// Draw line while dragging left mouse, create obstacle on release
		DrawBorders();
		Obstacles_Draw(&camera, obstacles);

		//Uint64 drawEnd = SDL_GetPerformanceCounter();

		presentScene();


		Profiler_End(&profiler, &stats, &titleStats, STAT_DRAW);

		Uint64 frameEnd = SDL_GetPerformanceCounter();

		Uint64 performanceFreq = SDL_GetPerformanceFrequency();

		Stats_UpdateTitle(
			&titleStats,
			app.window,
			deltaTime);


		double frameSeconds =
			(double)(frameEnd - frameStart) / performanceFreq;

		double targetFrameTime = 1.0 / 60.0;

		stats.totalFrames++;

		if (BENCHMARK_FRAMES > 0 && stats.totalFrames >= BENCHMARK_FRAMES)
		{
			exit(0);
		}

		Profiler_End(&profiler, &stats, &titleStats, STAT_FRAME_WORK);

		if (frameSeconds < targetFrameTime)
		{
			SDL_Delay((Uint32)((targetFrameTime - frameSeconds) * MS_PER_SECOND));
		}
		frameCount++;
	}

	return 0;
}


void printStats(void)
{
	printf("\n--- Simulation Stats ---\n");

#ifdef _DEBUG
	printf("Build: Debug\n");
#else
	printf("Build: Release\n");
#endif

	printf("Real type: %s\n", REAL_TYPE_NAME);
	printf("Boids: %d\n", BOID_COUNT);
	printf("POIs: %d\n", NUM_POI);
	printf("Seed: %d\n", RNG_SEED);
	printf("Benchmark Frames: %d\n", BENCHMARK_FRAMES);
	printf("Warm-up Frames: %d\n", WARMUP_FRAMES);
	printf("Runtime: %.2f sec\n", stats.runTime);

	for (int i = 0; i < STAT_COUNT; i++)
	{
		StatMetric* metric = &stats.metrics[i];

		if (metric->samples == 0 || metric->name == NULL)
		{
			continue;
		}

		printf("%s min/avg/max: %.4f / %.4f / %.4f\n",
			metric->name,
			metric->min,
			metric->sum / metric->samples,
			metric->max);
	}

	printf("\n------------------------\n");

	cleanup();
}

void DrawBoidsBatchedPoints(Camera* camera, Boid* boids, int numBoids, SimulationParameters* sim, bool rebuildPoints)
{
	static SDL_Point* points = NULL;
	static int capacity = 0;
	static int pointCount = 0;

	if (numBoids > capacity)
	{
		SDL_Point* newPoints = realloc(points, sizeof(SDL_Point) * numBoids);

		if (newPoints == NULL)
		{
			SDL_Log("Failed to allocate point batch");
			return;
		}

		points = newPoints;
		capacity = numBoids;
	}

	if (rebuildPoints)
	{
		for (int i = 0; i < numBoids; i++)
		{
			vec2 screen = WorldToScreen(camera, (vec2) { boids[i].x, boids[i].y });

			points[i].x = (int)screen.x;
			points[i].y = (int)screen.y;
		}

		pointCount = numBoids;
	}

	SDL_SetRenderDrawColor(
		app.renderer,
		sim->boidColor.r,
		sim->boidColor.g,
		sim->boidColor.b,
		sim->boidColor.a
	);

	SDL_RenderDrawPoints(app.renderer, points, pointCount);
}


void DrawBoidsBatchedPointsInterpolated(
	Camera* camera,
	const Boid* previous,
	const Boid* current,
	int numBoids,
	SimulationParameters* sim,
	real alpha,
	bool rebuildPoints)
{
	static SDL_Point* points = NULL;
	static int capacity = 0;
	static int pointCount = 0;

	if (numBoids > capacity)
	{
		SDL_Point* newPoints =
			realloc(points, sizeof(SDL_Point) * numBoids);

		if (newPoints == NULL)
		{
			SDL_Log("Failed to allocate point batch");
			return;
		}

		points = newPoints;
		capacity = numBoids;
	}

	if (rebuildPoints)
	{
		for (int i = 0; i < numBoids; i++)
		{
			real x =
				previous[i].x +
				(current[i].x - previous[i].x) * alpha;

			real y =
				previous[i].y +
				(current[i].y - previous[i].y) * alpha;

			vec2 screen =
				WorldToScreen(camera, (vec2) { x, y });

			points[i].x = (int)screen.x;
			points[i].y = (int)screen.y;
		}

		pointCount = numBoids;
	}

	SDL_SetRenderDrawColor(
		app.renderer,
		sim->boidColor.r,
		sim->boidColor.g,
		sim->boidColor.b,
		sim->boidColor.a
	);

	SDL_RenderDrawPoints(
		app.renderer,
		points,
		pointCount
	);
}

vec2 GetCameraInputDirection()
{
	vec2 dir = { 0, 0 };
	if (app.up) dir.y -= 1;
	if (app.down) dir.y += 1;
	if (app.left) dir.x -= 1;
	if (app.right) dir.x += 1;
	return vec_norm(dir);
}

void DrawBorders() {

	vec2 topLeft = WorldToScreen(&camera, (vec2) { 0, 0 });
	vec2 topRight = WorldToScreen(&camera, (vec2) { WORLD_WIDTH, 0 });
	vec2 bottomLeft = WorldToScreen(&camera, (vec2) { 0, WORLD_HEIGHT });
	vec2 bottomRight = WorldToScreen(&camera, (vec2) { WORLD_WIDTH, WORLD_HEIGHT });


	draw_line(topLeft.x, topLeft.y, topRight.x, topRight.y, (DrawColor) { 255, 255, 255, 255 });
	draw_line(topRight.x, topRight.y, bottomRight.x, bottomRight.y, (DrawColor) { 255, 255, 255, 255 });
	draw_line(bottomRight.x, bottomRight.y, bottomLeft.x, bottomLeft.y, (DrawColor) { 255, 255, 255, 255 });
	draw_line(bottomLeft.x, bottomLeft.y, topLeft.x, topLeft.y, (DrawColor) { 255, 255, 255, 255 });
}

void HandleObstacleInput(Obstacles** obstacles)
{
	static bool drawingLine = false;
	vec2 mouseWorldPos = ScreenToWorld(&camera, (vec2) { app.mouseX, app.mouseY });
	vec2 dragStartWorldPos = ScreenToWorld(&camera, (vec2) { app.dragStartX, app.dragStartY });
	vec2 dragEndWorldPos = ScreenToWorld(&camera, (vec2) { app.dragEndX, app.dragEndY });
	vec2 rMouseWorldPos = ScreenToWorld(&camera, (vec2) { app.rmouseX, app.rmouseY });
	vec2 prevRMouseWorldPos = ScreenToWorld(&camera, (vec2) { app.prevRMouseX, app.prevRMouseY });

	if (app.mouseDown)
	{
		draw_line(app.dragStartX, app.dragStartY,
			app.mouseX, app.mouseY,
			(DrawColor) {
			255, 255, 255, 255
		});

		drawingLine = true;
	}
	else if (drawingLine)
	{
		Obstacles_Add(obstacles,
			(real)dragStartWorldPos.x,
			(real)dragStartWorldPos.y,
			(real)dragEndWorldPos.x,
			(real)dragEndWorldPos.y);
		drawingLine = false;
	}

	if (app.rmouseDown)
	{
		draw_line(app.prevRMouseX, app.prevRMouseY,
			app.rmouseX, app.rmouseY,
			(DrawColor) {
			255, 0, 0, 255
		});

		Obstacles_EraseIntersecting(obstacles,
			prevRMouseWorldPos.x,
			prevRMouseWorldPos.y,
			rMouseWorldPos.x,
			rMouseWorldPos.y);
	}
}


void RunFlockWorkers(
	WorkerPool* workerpool,
	UniformGrid* grid,
	int boidCount,
	Boid** boids,
	Boid** boidsNext,
	Obstacles* obstacles,
	SimulationParameters* sim,
	PointOfInterest* pointsOfInterest,
	int poiCount,
	real deltaTime
)
{
	FlockJob job = initFlockJob(
		0,
		boidCount,
		*boids,
		*boidsNext,
		grid,
		obstacles,
		sim,
		pointsOfInterest,
		poiCount,
		deltaTime
	);

	WorkerPool_Run(
		workerpool,
		boidCount,
		256,
		workerpool->numThreads,
		FlockJob_Run,
		&job
	);

	Boid* tmp = *boids;
	*boids = *boidsNext;
	*boidsNext = tmp;
}