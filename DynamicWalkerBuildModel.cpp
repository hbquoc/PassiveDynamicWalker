//==============================================================================
//The OpenSim Main header must be included in all files
#include <OpenSim/OpenSim.h>
// Set the namespace to shorten the declarations
// Note: Several classes appear in both namespaces and require using the full name
using namespace OpenSim;
using namespace SimTK;
//______________________________________________________________________________
/**
 *
 */
int main()
{
    try {
        // Code to the construct the model will go here
        // Section: Setup
        // Define key model variables
        double pelvisWidth = 0.20, thighLength = 0.40, shankLength = 0.435;
 
        // Create an OpenSim Model
        Model osimModel = Model();
        osimModel.setName("DynamicWalkerModel");
 
        // Get a reference to the ground object
        OpenSim::Body& ground = osimModel.getGroundBody();
 
        // Define the acceleration of gravity
        osimModel.setGravity(Vec3(0, -9.80665, 0));
 
        /// Section: Create the Platform
		double mass = 1;
		 
		// Location of the center of mass from the body origin, expressed in the body frame
		Vec3 comLocInBody(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		Inertia bodyInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* platform = new OpenSim::Body("Platform",
													mass, comLocInBody, bodyInertia);
		 
		// Section: Create the Platform Joint
		// Create the joint connecting the platform to the ground
		Vec3 locationInParent(0.0, 0.0, 0.0);
		Vec3 orientationInParent(0.0, 0.0, 0.0);
		Vec3 locationInChild(0.0, 0.0, 0.0);
		Vec3 orientationInChild(0.0, 0.0, 0.0);
		PinJoint *platformToGround = new PinJoint("PlatformToGround",
				ground, locationInParent, orientationInParent,
				*platform, locationInChild, orientationInChild);
		 
		// Section: Set the properties of the coordinates that define the joint
		// A pin joint consists of a single coordinate describing a change in
		// orientation about the Z axis
		CoordinateSet& platformCoords = platformToGround->upd_CoordinateSet();
		platformCoords[0].setName("platform_rz");
		double rotRangePlatform[2] = {-Pi/2.0, 0};
		platformCoords[0].setRange(rotRangePlatform);
		platformCoords[0].setDefaultValue(convertDegreesToRadians(-10.0));
		platformCoords[0].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		platform->addDisplayGeometry("box.vtp");
		platform->updDisplayer()->setScaleFactors(Vec3(1, 0.05, 1));
		
		// Add the platform to the Model
		osimModel.addBody(platform);
		
		//-------------------------------------------------------------------------------//
		
		// Section: Create the Pelvis
		mass = 1;
		 
		// Location of the center of mass from the body origin expressed in body frame
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* pelvis = new OpenSim::Body("Pelvis", mass, comLocInBody, bodyInertia);
		 
		// Create the joint which connects the pelvis to the platform
		locationInParent    = Vec3(0.0, 0.0, 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		locationInChild     = Vec3(0.0, 0.0, 0.0);
		orientationInChild  = Vec3(0.0, 0.0, 0.0);
		FreeJoint *pelvisToPlatform = new FreeJoint("PelvisToPlatform",
				*platform, locationInParent, orientationInParent,
				*pelvis, locationInChild, orientationInChild);
		 
		// A Free joint has six coordinates, in the following order:
		//     rot_x, rot_y, rot_z, trans_x, trans_y, trans_z
		// Set the properties of the coordinates that define the joint
		CoordinateSet& pelvisJointCoords = pelvisToPlatform->upd_CoordinateSet();
		pelvisJointCoords[0].setName("pelvis_rx");
		double rotRangePelvis[2] = {-Pi, Pi};
		pelvisJointCoords[0].setRange(rotRangePelvis);
		pelvisJointCoords[0].setDefaultValue(0);
		pelvisJointCoords[0].setDefaultLocked(true);

		pelvisJointCoords[1].setName("pelvis_ry");
		pelvisJointCoords[1].setRange(rotRangePelvis);
		pelvisJointCoords[1].setDefaultValue(0);
		pelvisJointCoords[1].setDefaultLocked(true);
		
		pelvisJointCoords[2].setName("pelvis_rz");
		pelvisJointCoords[2].setRange(rotRangePelvis);
		pelvisJointCoords[2].setDefaultValue(0);
		pelvisJointCoords[2].setDefaultLocked(true);
		
		pelvisJointCoords[3].setName("pelvis_tx");
		double transRangePelvis[2] = {-10, 10};
		pelvisJointCoords[3].setRange(transRangePelvis);
		pelvisJointCoords[3].setDefaultValue(0);
		
		pelvisJointCoords[4].setName("pelvis_ty");
		transRangePelvis = {-1, 2};
		pelvisJointCoords[4].setRange(transRangePelvis);
		pelvisJointCoords[4].setDefaultValue(1);
		
		pelvisJointCoords[5].setName("pelvis_tz");
		transRangePelvis = {-1, 1};
		pelvisJointCoords[5].setRange(transRangePelvis);
		pelvisJointCoords[5].setDefaultValue(0);
		pelvisJointCoords[5].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		pelvis->addDisplayGeometry("sphere.vtp");
		pelvis->updDisplayer()->setScaleFactors(
				Vec3(pelvisWidth/2.0, pelvisWidth/2.0, pelvisWidth));
		 
		// Add the pelvis to the Model
		osimModel.addBody(pelvis);
		
		//--------------------------------------------------------------------------
		
		// Section: Create the left thigh
		mass = 1;
		 
		// Location of the center of mass from the body origin expressed in body frame
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* leftthigh = new OpenSim::Body("LeftThigh", mass, comLocInBody, bodyInertia);
		 
		// Create the joint which connects the left thigh to the pelvis
		locationInParent    = Vec3(0.0, pelvisWidth/2.0, 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		locationInChild     = Vec3(0.0, thighLength/2.0, 0.0);
		orientationInChild  = Vec3(0.0, 0.0, 0.0);
		PinJoint *leftthighToPelvis = new PinJoint("LeftThighToPelvis",
				*pelvis, locationInParent, orientationInParent,
				*leftthigh, locationInChild, orientationInChild);
		 
		// Left Hip Pin Joint
		CoordinateSet& lefthipJointCoords = leftthighToPelvis->upd_CoordinateSet();
		lefthipJointCoords[0].setName("LHip_rz");
		double rotRangeHip[2] = {-100, 100};
		lefthipJointCoords[0].setRange(convertDegreesToRadians(rotRangeHip));
		lefthipJointCoords[0].setDefaultValue(convertDegreesToRadians(-10.0));
		lefthipJointCoords[0].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		leftthigh->addDisplayGeometry("sphere.vtp");
		leftthigh->updDisplayer()->setScaleFactors(
				Vec3(thighLength/10.0, thighLength/10.0, thighLength));
		 
		// Add the left thigh to the Model
		osimModel.addBody(leftthigh);
		
		//--------------------------------------------------------------------------------------
		
		// Section: Create the left shank
		mass = 1;
		 
		// Location of the center of mass from the body origin expressed in body frame
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* leftshank = new OpenSim::Body("LeftShank", mass, comLocInBody, bodyInertia);
		 
		// Create the joint which connects the left shank to the left thigh
		locationInParent    = Vec3(0.0, -thighLength/2.0, 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		locationInChild     = Vec3(0.0, shankLength/2.0, 0.0);
		orientationInChild  = Vec3(0.0, 0.0, 0.0);
		PinJoint *leftshankToThigh = new PinJoint("LeftShankToThigh",
				*leftthigh, locationInParent, orientationInParent,
				*leftshank, locationInChild, orientationInChild);
		 
		// Left Knee Pin Joint
		CoordinateSet& leftkneeJointCoords = leftshankToThigh->upd_CoordinateSet();
		leftkneeJointCoords[0].setName("LKnee_rz");
		double rotRangeKnee[2] = {-100, 0};
		leftkneeJointCoords[0].setRange(convertDegreesToRadians(rotRangeKnee));
		leftkneeJointCoords[0].setDefaultValue(convertDegreesToRadians(-30.0));
		leftkneeJointCoords[0].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		leftshank->addDisplayGeometry("sphere.vtp");
		leftshank->updDisplayer()->setScaleFactors(
				Vec3(shankLength/10.0, shankLength/10.0, shankLength));
		 
		// Add the left shank to the Model
		osimModel.addBody(leftshank);
		
		//-----------------------------------------------------------------------------------------
		
		// Section: Create the right thigh
		mass = 1;
		 
		// Location of the center of mass from the body origin expressed in body frame
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* rightthigh = new OpenSim::Body("RightThigh", mass, comLocInBody, bodyInertia);
		 
		// Create the joint which connects the right thigh to the pelvis
		locationInParent    = Vec3(0.0, -pelvisWidth/2.0, 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		locationInChild     = Vec3(0.0, thighLength/2.0, 0.0);
		orientationInChild  = Vec3(0.0, 0.0, 0.0);
		PinJoint *rightthighToPelvis = new PinJoint("RightThighToPelvis",
				*pelvis, locationInParent, orientationInParent,
				*rightthigh, locationInChild, orientationInChild);
		 
		// Right Hip Pin Joint
		CoordinateSet& righthipJointCoords = rightthighToPelvis->upd_CoordinateSet();
		righthipJointCoords[0].setName("RHip_rz");
		righthipJointCoords[0].setRange(convertDegreesToRadians(rotRangeHip));
		righthipJointCoords[0].setDefaultValue(convertDegreesToRadians(30.0));
		righthipJointCoords[0].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		rightthigh->addDisplayGeometry("sphere.vtp");
		rightthigh->updDisplayer()->setScaleFactors(
				Vec3(thighLength/10.0, thighLength/10.0, thighLength));
		 
		// Add the right thigh to the Model
		osimModel.addBody(rightthigh);
		
		//---------------------------------------------------------------------------
		
		// Section: Create the right shank
		mass = 1;
		 
		// Location of the center of mass from the body origin expressed in body frame
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		 
		// Inertia of the body expressed in the body frame
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
		 
		// Create the body
		OpenSim::Body* rightshank = new OpenSim::Body("RightShank", mass, comLocInBody, bodyInertia);
		 
		// Create the joint which connects the right shank to the right thigh
		locationInParent    = Vec3(0.0, -thighLength/2.0, 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		locationInChild     = Vec3(0.0, shankLength/2.0, 0.0);
		orientationInChild  = Vec3(0.0, 0.0, 0.0);
		PinJoint *rightshankToThigh = new PinJoint("RightShankToThigh",
				*rightthigh, locationInParent, orientationInParent,
				*rightshank, locationInChild, orientationInChild);
		 
		// Right Knee Pin Joint
		CoordinateSet& rightkneeJointCoords = rightshankToThigh->upd_CoordinateSet();
		rightkneeJointCoords[0].setName("RKnee_rz");
		rightkneeJointCoords[0].setRange(convertDegreesToRadians(rotRangeKnee));
		rightkneeJointCoords[0].setDefaultValue(convertDegreesToRadians(-30.0));
		rightkneeJointCoords[0].setDefaultLocked(true);
		 
		// Add and scale model for display in GUI
		rightshank->addDisplayGeometry("sphere.vtp");
		rightshank->updDisplayer()->setScaleFactors(
				Vec3(shankLength/10.0, shankLength/10.0, shankLength));
		 
		// Add the right shank to the Model
		osimModel.addBody(rightshank);
 
        // **********  END CODE  **********
 
        // TODO: Construct ContactGeometry and HuntCrossleyForces Here
        // ********** BEGIN CODE **********
 
 
        // **********  END CODE  **********
 
        // TODO: Construct CoordinateLimitForces Heres
        // ********** BEGIN CODE **********
 
 
        // **********  END CODE  **********
 
        // Save the model to a file
        osimModel.print("DynamicWalkerModel.osim");
    }
    catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (SimTK::Exception::Base ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }
    std::cout << "OpenSim example completed successfully" << std::endl;
    std::cout << "Press return to continue" << std::endl;
    std::cin.get();
    return 0;
 } 