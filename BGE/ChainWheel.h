#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class ChainWheel :
		public Game
	{
	private:

	public:
		shared_ptr<PhysicsController> chain;
		shared_ptr<PhysicsController> craneBlock;
		ChainWheel(void);
		~ChainWheel(void);
		bool Initialise();
		void Update();
		void Cleanup();
		shared_ptr<PhysicsController> CreateChain(int numOfBlocks, glm::vec3 topBlockPosition, float width, float height, float depth);
	};
}
