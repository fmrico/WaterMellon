/*
 * Particle.h
 *
 *  Created on: 20/09/2015
 *      Author: paco
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_


class Particle {

public:
	Particle(): coord(), p(0.0) {};

	Particle(const Particle& part) {
		coord = part.coord;
		p = part.p;
	}

	Particle& operator= (const Particle& part)
	{
		coord = part.coord;
		p = part.p;

		return *this;
	}

	bool operator<(const Particle& part) const
	{
		return p<part.p;
	}

	geometry_msgs::Pose coord;
	float p;

};


#endif /* PARTICLE_H_ */
