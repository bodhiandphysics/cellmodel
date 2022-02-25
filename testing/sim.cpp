#include <"sim.hpp">

using glm;
using PositionList = std::vector<Positions>;
using LinConstraintList = std::vector<LinConstraint>;
using AngConstraintList = std::vector<AngConstraint>

static const int NUMITERS = 50; //guessing?


void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, double deltat) {

	//update prodicted positions

	for (auto position: positions) {

		position.predict = position.position + position.velocity * deltat + deltat*deltat*(1/position.mass)*position.getforce();
	}


	for (auto linconstraint: linconstraints) linconstraint.lambda = 0;
	for (auto angconstraint: angconstraints) angconstraint.lambda = 0


	// for each constraint use interated solver to solve constraint

	for (int i = 0; i < NUMITERS; i++) {

	// first linear constraints

		for (auto linconstraint: linconstraints) {

			float x1 = linconstraint.pos1->predict.x;
			float x2 = linconstraint.pos2->predict.x;
			float y1 = linconstraint.pos1->predict.y;
			float y2 = linconstraint.pos2->predict.y;
			float m1 = linconstaint.pos1->mass;
			float m2 = linconstraint.pos2->mass;


			 modalph = linconstraint.alpha/(deltat*deltat);

			// C(p)
			auto constraintval = (x1 - x2) * (x1 - x2)
								 + (y1 - y2) * (y1 - y2)
								 - linconstraint.restlength * linconstraint.restlength;

			// delC * 1/m * delCT					 

			float cinnerval = (4/m1) * ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) + (4/m2) * ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

			float deltal = (-constraintval + modalph*linconstraint.lamda) / (cinnerval + modalph);

			float deltax1 = (2/m1) * (x1-x2) * deltal;
			float deltax2 = -(2/m2) * (x1-x2) * deltal;
			float deltay1 = (2/m1) * (y1-y2) * deltal;
			float deltay2 = -(2/m2) * (y1-y2) * deltal;

			linconstraint.lambda += deltal;

			if (!pos1->pinned) {
				linconstraint->pos1.predictx += deltax1;
				linconstraint->pos1.predicty += deltay1;
			}
			if (!pos2->pinned) {
				linconstraint->pos2.predictx += deltax2;
				linconstraint->pos2.predicty += deltay2;
			}
		} 



		// Now do angular constraints


		for (auto angconstraint: angularconstraints) {

			float x1 = angconstraint.pos1->predict.x;
			float x2 = angconstraint.pos2->predict.x;
			float xc = angconstraint.posc->predict.x;
			float y1 = angconstraint.pos1->predict.y;
			float y2 = angconstraint.pos2->predict.y;
			float yc = angconstraint.posc->predict.y;
			float s1x = x1 - xc;
			float s1y = y1 - yc;
			float s2x = x2 - xc;
			float s2y = y2 - yc;
			float m1 = angconstraint.pos1->mass;
			float m2 = angconstraint.pos2->mass;
			float mc = angconstraint.posc->mass;

			float modalph = angconstraint.alpha/(deltat*deltat);

			float s1s2 = s1x*s2x + s1y*s2y;
			float s1norm2 = s1x*s1x + s1y*s2y;
			float s2norm2 = s2x*s2x + s2y*s2y;
			float normproduct = s1norm2 * s2norm2

			float theta = s1s2/sqrt(s1norm2*s2norm2);

			float constraintval = acos(theta) - angconstraint.theta0; 

			float darccos = -1 / sqrt(1 - theta*theta);

			float delcx1 = darccos * (sqrt(normproduct)*s2x + s1s2*pow(normproduct, -1.5)*s2norm2*s1x) / normproduct;
			float delcy1 = darccos * (sqrt(normproduct)*s2y + s1s2*pow(normproduct, -1.5)*s2norm2*s1y) / normproduct;
			float delcx2 = darccos * (sqrt(normproduct)*s1x + s1s2*pow(normproduct, -1.5)*s1norm2*s2x) / normproduct;
			float delcy2 = darccos * (sqrt(normproduct)*s1y + s1s2*pow(normproduct, -1.5)*s1norm2*s2y) / normproduct;

			float delcxc = (darccos / normproduct) * (sqrt(normproduct) * (2*xc - x1 - x2) - (s1s2*pow(normproduct, -0.5)*
																							(s1norm2*s2x + s2norm2*s1x)));

			float delcyc = (darccos / normproduct) * (sqrt(normproduct) * (2*yc - y1 - y2) - (s1s2*pow(normproduct, -0.5)*
																							(s1norm2*s2y + s2norm2*s1y)));

			float cinnerval = (delcx1*delcx1 + delcy1*delcy1)/m1 + 
							 (delcx2*delcx2 + delcy2*delcy2)/m2 + 
							 (delcxc*delcxc + delcyc*delcyc)/mc;

			float deltal = (-constraintval + modalph*linconstraint.lamda) / (constraintval + modalph);

			float deltax1 = (1/m1)*delcx1*deltal;
			float deltax2 = (1/m2)*delcx2*deltal;
			float deltay1 = (1/m1)*delcy1*deltal;
			float deltay2 = (1/m2)*delcy2*deltal;
			float deltaxc = (1/mc)*delcxc*deltal;
			float deltayc = (1/mc)*delcyc*deltal;


			angconstraint.lambda += deltal;

			if (!pos1->pinned) {
				angconstraint.pos1->predict.x += deltax1;
				angconstraint.pos1->predict.y += deltay1;
			}
			if (!pos2->pinned) {
				angconstraint.pos2->predict.x += deltax2;
				angconstraint.pos2->predict.y += deltay2;
			}
			
			if (!posc->pinned) {
				angconstraint.posc->predict.x += deltaxc;
				angconstrain.posc->predict.y += deltayc;
			}

		}

	}
	// update positions and velcities

	for (auto position: positions) {

		if (!position->pinned) {
			position.velocity.x = (position.predict.x - position.position.x) / deltat;
			position.velocity.y = (position.predict.y - position.position.y) / deltat;
			position.velocity.z = (position.predict.z - position.position.z) / deltat;
			position.position.x = position.predict.x;
			position.postition.y = position.predict.y;
			position.position.z = position.predict.z;
		}
	}

}