#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "ChainWheel.h"

using namespace BGE;

ChainWheel::ChainWheel() {

}

ChainWheel::~ChainWheel() {

}

bool ChainWheel::Initialise() {
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	Game::setGravity(glm::vec3(0, -9.81f, 0));

	chain = CreateChain(5, glm::vec3(10, 19, 1), 6, 1, 6);

	if (!Game::Initialise()) {
		return false;
	}

	return true;
}

void ChainWheel::Update() {
	Game::Update();
}

void ChainWheel::Cleanup() {
	Game::Cleanup();
}

shared_ptr<PhysicsController> ChainWheel::CreateChain(int numOfBlocks, glm::vec3 topBlockPosition, float width, float height, float depth) {

	shared_ptr<PhysicsController> currentBlock;
	shared_ptr<PhysicsController> previousBlock;
	shared_ptr<PhysicsController> firstBlock;
	shared_ptr<PhysicsController> lastBlock;
	shared_ptr<PhysicsController> wheel;
	shared_ptr<PhysicsController> firstWheel;
	shared_ptr<PhysicsController> lastWheel;
	glm::quat q = glm::quat();
	glm::vec3 offset;
	btHingeConstraint * wheelHinge;

	float gapHeight = depth / 2.0f;
	float gapWidth = width / 1.5f;

	for (int i = 0; i < numOfBlocks; i += 1) {
		currentBlock = physicsFactory->CreateBox(width, height, depth, topBlockPosition + glm::vec3((width * i), 0, 0), glm::quat());
		if (previousBlock != NULL) {
			btPoint2PointConstraint * backConnector = new btPoint2PointConstraint(*previousBlock->rigidBody, *currentBlock->rigidBody, btVector3(-gapWidth, 0, depth), btVector3(gapWidth, 0, depth));
			dynamicsWorld->addConstraint(backConnector);
			btPoint2PointConstraint * frontConnector = new btPoint2PointConstraint(*previousBlock->rigidBody, *currentBlock->rigidBody, btVector3(-gapWidth, 0, -depth), btVector3(gapWidth, 0, -depth));
			dynamicsWorld->addConstraint(frontConnector);
		}
		if (i == 0) {
			firstBlock = currentBlock;
			offset = glm::vec3((width / 2.0f + 1.0f), 0, 0);
			wheel = physicsFactory->CreateSphere(2, topBlockPosition + offset, q);
			wheelHinge = new btHingeConstraint(*currentBlock->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
			wheelHinge->enableAngularMotor(true, -10, 100);
			dynamicsWorld->addConstraint(wheelHinge);
			firstWheel = wheel;
		}
		else if (i == numOfBlocks - 1) {
			lastBlock = currentBlock;
			offset = glm::vec3(-(width / 2.0f + 1.0f), 0, 0);
			wheel = physicsFactory->CreateSphere(2, topBlockPosition + offset, q);
			wheelHinge = new btHingeConstraint(*currentBlock->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
			wheelHinge->enableAngularMotor(true, -10, 100);
			dynamicsWorld->addConstraint(wheelHinge);
			previousBlock = currentBlock;
			lastWheel = wheel;
		}
		offset = glm::vec3(0, 0, (depth / 2) + 1.0f);
		wheel = physicsFactory->CreateSphere(2, topBlockPosition + offset, q);
		wheelHinge = new btHingeConstraint(*currentBlock->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
		wheelHinge->enableAngularMotor(true, -10, 100);
		dynamicsWorld->addConstraint(wheelHinge);

		offset = glm::vec3(0, 0, -((depth / 2) + 1.0f));
		wheel = physicsFactory->CreateSphere(2, topBlockPosition + offset, q);
		wheelHinge = new btHingeConstraint(*currentBlock->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
		wheelHinge->enableAngularMotor(true, -10, 100);
		dynamicsWorld->addConstraint(wheelHinge);
		previousBlock = currentBlock;
	}

	wheelHinge = new btHingeConstraint(*firstWheel->rigidBody, *lastWheel->rigidBody, btVector3(4, 0, 0), btVector3(-4, 0, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(wheelHinge);


	return firstBlock;
}
