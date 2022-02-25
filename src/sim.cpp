#include "sim.hpp"
#include <cstdlib>
#include <iostream>

float originallength = 20.f;



float smallest_distance2_pv (glm::vec2 p, glm::vec2 v1, glm::vec2 v2) { // note produces sauare distance!

	glm::vec2 a = p - v1;
	glm::vec2 b = p - v2;
	glm::vec2 c = v1 - v2;

	if (glm::dot(a, c) < 0)
		return glm::length(a);
	if (glm::dot(b, c) < 0)
		return glm::length(b);

	return (dot(a,a) + dot(b,b) - dot(c,c))/ 2.0f;
}

void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, float deltat, bool gravity, bool use_bounds, bool use_collisions) {

	CollisionConstraintList colconstraints;

	//update prodicted positions

	for (auto &position: positions) {

		position.predict = position.position + position.velocity * deltat;
		if (gravity) position.predict += vec2(0.f, -9.8f)*deltat*deltat; //gravity
	}

	if (use_collisions){
		for (auto position = positions.begin(); position != positions.end(); position++) {

			float max_distance = FLT_MAX;
			LinConstraintList::iterator max_constraint = linconstraints.begin(); 

			for (auto linconstraint = linconstraints.begin(); linconstraint != linconstraints.end(); linconstraint++) {

				if (position == linconstraint->pos1 || position == linconstraint->pos2) // don't collide with adjoining segs
				 continue;

				const glm::vec2 pos1 = linconstraint->pos1->position;
				const glm::vec2 pos2 = linconstraint->pos2->position;
				float min_length = smallest_distance2_pv( position->position, pos1, pos2);
				if (min_length < linconstraint->restlength*2.f) {
					colconstraints.emplace_back(position, linconstraint);
				}
					// max_distance = min_length;
					// max_constraint = linconstraint;
			}
		}
	}

		


	for (auto &linconstraint: linconstraints) linconstraint.lambda = 0; // zero the lambdas for every frame
	for (auto &angconstraint: angconstraints) angconstraint.lambda = 0; // note that colconstraints don't need lambda

	// for each constraint use interated solver to solve constraint

	bool done = false;
	float convergence_distance = 0;
	float convergence_distance_last = 0; 
	float convergence_max = 0;
	float allow_flag = true;

	while (!done) {
		convergence_distance = 0; 
		convergence_max = 0;


	// first linear constraints

		for (auto &linconstraint: linconstraints) {

			float x1 = linconstraint.pos1->predict.x;
			float x2 = linconstraint.pos2->predict.x;
			float y1 = linconstraint.pos1->predict.y;
			float y2 = linconstraint.pos2->predict.y;
			float m1 = linconstraint.pos1->mass;
			float m2 = linconstraint.pos2->mass;


			float modalph = linconstraint.alpha/(deltat*deltat);

			// C(p)

			float distance =  sqrt((x1 - x2) * (x1 - x2)
								 + (y1 - y2) * (y1 - y2));

			if (distance < .0000001f) distance = .0000001f;

			float distance2 = distance * distance;
			auto constraintval = distance - linconstraint.restlength;
								
			// delC * 1/m * delCT					 

			float cinnerval = (1.f/(distance2*m1)) * ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) + 
							  (1.f/(distance2*m2)) * ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

			if (isnan(cinnerval)) cinnerval = 0.f;

			float deltal = (-constraintval - modalph*linconstraint.lambda) / (cinnerval + modalph);

			float deltax1 = (1.f/(distance*m1)) * (x1-x2) * deltal;
			float deltax2 = -(1.f/(distance*m2)) * (x1-x2) * deltal;
			float deltay1 = (1.f/(distance*m1)) * (y1-y2) * deltal;
			float deltay2 = -(1.f/(distance*m2)) * (y1-y2) * deltal;

			linconstraint.lambda += deltal;

			if (linconstraint.pos1->pinned) {
				deltax1 = 0.f;
				deltay1 = 0.f;
			}
			if (linconstraint.pos2->pinned) {
				deltax2 = 0.f;
				deltay2 = 0.f;
			}

			linconstraint.pos1->predict.x += deltax1;
			linconstraint.pos1->predict.y += deltay1;
			linconstraint.pos2->predict.x += deltax2;
			linconstraint.pos2->predict.y += deltay2;

			convergence_distance += deltax1*deltax1 + deltay1*deltay1 + deltax2*deltax2 + deltay2*deltay2;
			convergence_max = std::max({convergence_max, deltax1, deltay1, deltax2, deltay2});

		} 



		// Now do angular constraints


		for (auto &angconstraint: angconstraints) {

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

			float s1ds2 = s1x*s2x + s1y*s2y;
			float s1xs2 = s1x*s2y - s1y*s2x;

			float constraintval = atan2(s1xs2,s1ds2) - angconstraint.theta0;

			float datan2 = 1.f / (s1ds2*s1ds2 + s1xs2*s1xs2); 

			float delcx1 = datan2 * ((s1ds2*s2y) - (s1xs2*s2x));
			float delcx2 = datan2 * (-(s1ds2*s1y) - (s1xs2*s1x));
			
			float delcy1 = datan2 * (-(s1ds2*s2x) - (s1xs2*s2y));
			float delcy2 = datan2 * ((s1ds2*s1x) - (s1xs2*s1y));
			
			float delcxc = datan2 * ((s1ds2*(s2y-s1y)) - (-s1xs2*(s1x+s2x)));
			float delcyc = datan2 * ((s1ds2*(s2x-s1x)) - (-s1xs2*(s1y+s2y)));

			float cinnerval = (1.f/m1)*(delcx1*delcx1 + delcy1*delcy1) +
							  (1.f/m2)*(delcx2*delcx2 + delcy2*delcy2) +
							  (1.f/mc)*(delcxc*delcxc + delcyc*delcyc);


			float deltal = (-constraintval - modalph*angconstraint.lambda) / (cinnerval + modalph);



			float deltax1 = (1.f/m1)*delcx1*deltal;
			float deltax2 = (1.f/m2)*delcx2*deltal;
			float deltay1 = (1.f/m1)*delcy1*deltal;
			float deltay2 = (1.f/m2)*delcy2*deltal;
			float deltaxc = (1.f/mc)*delcxc*deltal;
			float deltayc = (1.f/mc)*delcyc*deltal;



			angconstraint.lambda += deltal;

			if (angconstraint.pos1->pinned) {
				deltax1 = 0.f;
				deltay1 = 0.f;
			}
			if (angconstraint.pos2->pinned) {
				deltax2 = 0.f;
				deltay2 = 0.f;
			}
			
			if (angconstraint.posc->pinned) {
				deltaxc = 0.f;
				deltayc = 0.f;
			}

			angconstraint.pos1->predict.x += deltax1;
			angconstraint.pos1->predict.y += deltay1;
			angconstraint.pos2->predict.x += deltax2;
			angconstraint.pos2->predict.y += deltay2;
			angconstraint.posc->predict.x += deltaxc;
			angconstraint.posc->predict.y += deltayc;

			convergence_distance += deltax1*deltax1 + deltay1*deltay1 + deltax2*deltax2 + deltay2*deltay2 + deltaxc*deltaxc + deltayc*deltayc;
			convergence_max = std::max({convergence_max, deltax1, deltay1, deltax2, deltay2, deltaxc, deltayc});

		}


		// Now do the collision constraints

		for (auto &colconstraint: colconstraints) {

			glm::vec2 pc = colconstraint.pos->predict;
			glm::vec2 p1 = colconstraint.segment->pos1->predict;
			glm::vec2 p2 = colconstraint.segment->pos2->predict;
			glm::vec2 s1 = p1 - pc;
			glm::vec2 s2 = p2 - pc;
			glm::vec2 s_const = colconstraint.segment->pos2->predict - colconstraint.segment->pos1->predict;

			float m1 = colconstraint.segment->pos1->mass;
			float m2 = colconstraint.segment->pos2->mass;
			float mc = colconstraint.pos->mass;
			float area_sign = colconstraint.area_sign;


			float constraintval = (s1.x*s2.y - s1.y*s2.x); 

			if (constraintval * area_sign > 0) continue; // the area is the same sine than zero;

			else { 

				float delcx1 =  s2.y;
				float delcx2 =  -s1.y;
				float delcxc =  -(s2.y - s1.y);
				float delcy1 =  -s2.x;
				float delcy2 =  s1.x;
				float delcyc =  -(s1.x - s2.x);

				float cinnerval = (1.f/m1)*(delcx1*delcx1 + delcy1*delcy1) +
							  	  (1.f/m2)*(delcx2*delcx2 + delcy2*delcy2) +
								  (1.f/mc)*(delcxc*delcxc + delcyc*delcyc);

				float deltal = -constraintval / cinnerval; // note modalph = 0 since the constraint is tight;

				float deltax1 = (1.f/m1)*delcx1*deltal;
				float deltax2 = (1.f/m2)*delcx2*deltal;
				float deltay1 = (1.f/m1)*delcy1*deltal;
				float deltay2 = (1.f/m2)*delcy2*deltal;
				float deltaxc = (1.f/mc)*delcxc*deltal;
				float deltayc = (1.f/mc)*delcyc*deltal;

				if (colconstraint.segment->pos1->pinned) {
					deltax1 = 0.f;
					deltay1 = 0.f;
				}
				if (colconstraint.segment->pos2->pinned) {
					deltax2 = 0.f;
					deltay2 = 0.f;
				}
			
				if (colconstraint.pos->pinned) {
					deltaxc = 0.f;
					deltayc = 0.f;
				}

				colconstraint.pos->predict.x += deltaxc;
				colconstraint.pos->predict.y += deltayc;
				colconstraint.segment->pos1->predict.x += deltax1;
				colconstraint.segment->pos1->predict.y += deltay1;
				colconstraint.segment->pos2->predict.x += deltax2;
				colconstraint.segment->pos2->predict.y += deltay2;



				convergence_distance += deltax1*deltax1 + deltay1*deltay1 + deltax2*deltax2 + deltay2*deltay2 + deltaxc*deltaxc + deltayc*deltayc;
				convergence_max = std::max({convergence_max, deltax1, deltay1, deltax2, deltay2, deltaxc, deltayc});
			}
		}

		if ((convergence_distance - convergence_distance_last) <= .01)
			done = true;
		else {
			done = false;
			convergence_distance_last = convergence_distance;
		}
	}

	// update positions and velcities
	for (auto &position: positions) {

		if (!position.pinned) {
			position.velocity.x = (position.predict.x - position.position.x) / deltat;
			position.velocity.y = (position.predict.y - position.position.y) / deltat;
			position.position.x = position.predict.x;
			position.position.y = position.predict.y;



			if (use_bounds) {
				if (position.position.x < -originallength/2) {
			
					// position.velocity.x = -position.velocity.x; 
					position.position.x = -originallength/2;
				}
	
				if (position.position.x > originallength/2) {
	
					// position.velocity.x = -position.velocity.x; 
					position.position.x = originallength/2;
				}
	
				if (position.position.y < -originallength/2) {
	
					// position.velocity.y = -position.velocity.y; 
					position.position.y = -originallength/2;
				}
	
				if (position.position.y > originallength/2) {
	
					// position.velocity.y = -position.velocity.y; 
					position.position.y = originallength/2;
				}
			}
		}
	}

	for (auto &constraint: linconstraints){
         constraint.restlength += (1.f/linconstraints.size())*deltat; // add some growth
       } 
}