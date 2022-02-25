#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <optional>

using namespace glm;

static const int NUMITERS = 300; //guessing?



struct Position {

	glm::vec2 position;
	glm::vec2 predict;
	glm::vec2 velocity = vec2(0,0);
	bool pinned = false; 


	float mass = .001f;

	Position(glm::vec2 pos): position(pos), predict(pos) {}
	Position(glm::vec2 pos, bool ispinned): position(pos), predict(pos), pinned(ispinned) {}
	
};

using PositionList = std::vector<Position>;

struct AngConstraint {

	PositionList::iterator pos1, pos2, posc;

	float alpha;
	float lambda = 0;
	float theta0;

	AngConstraint(PositionList::iterator apos1, 
				  PositionList::iterator apos2, 
				  PositionList::iterator aposc, 
				  float analpha): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha) {

		vec2 s1 = apos1->position - aposc->position;
		vec2 s2 = apos2->position - aposc->position;
		float s1xs2 = s1.x*s2.y - s1.y*s2.x;
		float s1ds2 = dot(s1, s2);
		theta0 = atan2(s1xs2, s1ds2);
	};

	AngConstraint(PositionList::iterator apos1, 
				  PositionList::iterator apos2, 
				  PositionList::iterator aposc, 
				  float analpha, float atheta0): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha), theta0(atheta0) {}

};

using AngConstraintList = std::vector<AngConstraint>;


struct LinConstraint {

	PositionList::iterator pos1, pos2;

	float alpha;
	float lambda = 0;
	float restlength;
	float original_length;
	std::optional<AngConstraintList::iterator> begin_ang = std::nullopt; 
	std::optional<AngConstraintList::iterator> end_ang = std::nullopt; 


	LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha): pos1(apos1), pos2(apos2), alpha(analpha) {

		restlength = glm::length(pos1->position - pos2->position);
		original_length = length(apos2->position - apos1->position);
	}

	LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha, float arestlength): pos1(apos1), pos2(apos2), alpha(analpha), restlength(arestlength) {
		original_length = length(apos2->position - apos1->position);
	}




	float currentlength() {return length(pos1->position - pos2->position);}
	

};

using LinConstraintList = std::vector<LinConstraint>;


struct CollisionConstraint {

	PositionList::iterator pos;
	LinConstraintList::iterator segment;
	float area_sign;

	CollisionConstraint(PositionList::iterator apos, LinConstraintList::iterator asegment):
	pos(apos), segment(asegment) {

		glm::vec2 a =  asegment->pos1->position - apos->position;
		glm::vec2 b =  asegment->pos2->position - apos->position;
		float area = a.x*b.y - a.y*b.x;
		if (area > 0) area_sign = 1.f;
		else area_sign = -1.f;

	}
};

using CollisionConstraintList = std::vector<CollisionConstraint>;




void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, float deltat, bool gravity, bool use_bounds, bool use_collisions);