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
	Particle(): coord_(), p_(0.0) {};

	Particle(const Particle& part) {
		coord_ = part.coord_;
		p_ = part.p_;
	}

	Particle& operator= (const Particle& part)
	{
		coord_ = part.coord_;
		p_ = part.p_;

		return *this;
	}

	bool operator<(const Particle& part) const
	{
		return p_<part.p_;
	}

	geometry_msgs::Pose coord_;
	float p_;

};


#endif /* PARTICLE_H_ */
