#pragma once
#include <glm/glm.hpp>
#include <vector>


using PositionList = std::vector<Positions>;
using LinConstraintList = std::vector<LinConstraint>;
using AngConstraintList = std::vector<AngConstraint>;

struct Position {

	glm::vec2 position;
	glm::vec2 predict = position;
	glm::vec2 velocity = 0;
	bool pinned = false; 

	float mass;

	glm::vec3 getforce() {

		return 1; // change if nescessary;
	}

	Position(glm::vec2 pos): position(pos) {}
	Position(glm::vec2 pos, bool ispinned): position(pos), pinned(ispinned) {}
	
};

struct LinConstraint {

	PositionList::iterator pos1, pos2;

	float alpha;
	float lambda = 0;
	float restlength = length(pos1 - pos2);

	LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha): pos1(apos1), pos2(apos2), alpha(analpha) {}
	

};

struct AngConstraint {

	PosititionList::iterator pos1, pos2, posc;

	float alpha;
	float lambda = 0;
	float theta0 = acos(dot(pos1-posc, pos2 - posc));

	AngConstraint(PosititionList::iterator apos1, 
				  PosititionList::iterator apos2, 
				  PosititionList::iterator aposc, 
				  float analpha): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha) {};



};

void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, double deltat);