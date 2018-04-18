/**
 * Based on "Mosegaards Cloth Simulation Coding Tutorial" ( http://cg.alexandra.dk/2009/06/02/mosegaards-cloth-simulation-coding-tutorial/ )
 */
#define _USE_MATH_DEFINES
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <vector>
#include <iostream>
#include "TextResource.h"
#include <SDL2/SDL.h>

namespace cloth_3_2_core_profile {
/* Some physics constants */
#define DAMPING 0.01f // how much to damp the cloth simulation each frame
#define TIME_STEPSIZE2 0.5f*0.5f // how large time step each particle takes each frame
#define CONSTRAINT_ITERATIONS 15 // how many iterations of constraint satisfaction each frame (more is rigid, less is soft)

using namespace glm;

GLuint litShader;
GLuint unlitShader;
mat4 projection;
mat4 view;
vec4 lightPos0; // light position in eye space
vec4 lightPos1;
int w;
int h;
SDL_Window *mainwindow; /* Our window handle */
void reshape(int w, int h);

/* The particle class represents a particle of mass that can move around in 3D space*/
class Particle
{
private:
	bool movable; // can the particle move or not ? used to pin parts of the cloth

	float mass; // the mass of the particle (is always 1 in this example)
	vec3 pos; // the current position of the particle in 3D space
	vec3 old_pos; // the position of the particle in the previous time step, used as part of the verlet numerical integration scheme
	vec3 acceleration; // a vector representing the current acceleration of the particle
	vec3 accumulated_normal; // an accumulated normal (i.e. non normalized), used for OpenGL soft shading

public:
	Particle(vec3 pos) : pos(pos), old_pos(pos),acceleration(vec3(0,0,0)), mass(1), movable(true), accumulated_normal(vec3(0,0,0)){}
	Particle(){}

	void addForce(vec3 f)
	{
		acceleration += f/mass;
	}

	/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
	   The method is called by Cloth.time_step()
	   Given the equation "force = mass * acceleration" the next position is found through verlet integration*/
	void timeStep()
	{
		if(movable)
		{
			vec3 temp = pos;
			pos = pos + (pos-old_pos)*(1.0f-DAMPING) + acceleration*TIME_STEPSIZE2;
			old_pos = temp;
			acceleration = vec3(0,0,0); // acceleration is reset since it HAS been translated into a change in position (and implicitely into velocity)	
		}
	}

	vec3& getPos() {return pos;}

	void resetAcceleration() {acceleration = vec3(0,0,0);}

	void offsetPos(const vec3 v) { if(movable) pos += v;}

	void makeUnmovable() {movable = false;}

	void addToNormal(vec3 normal)
	{
		accumulated_normal += normalize(normal);
	}

	vec3& getNormal() { return accumulated_normal;} // notice, the normal is not unit length

	void resetNormal() {accumulated_normal = vec3(0,0,0);}

};

class Constraint
{
private:
	float rest_distance; // the length between particle p1 and p2 in rest configuration

public:
	Particle *p1, *p2; // the two particles that are connected through this constraint

	Constraint(Particle *p1, Particle *p2) :  p1(p1),p2(p2)
	{
		vec3 vec = p1->getPos()-p2->getPos();
		rest_distance = length(vec);
	}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by Cloth.time_step() many times per frame*/
	void satisfyConstraint()
	{
		vec3 p1_to_p2 = p2->getPos()-p1->getPos(); // vector from p1 to p2
		float current_distance = length(p1_to_p2); // current distance between p1 and p2
		vec3 correctionVector = p1_to_p2*(1 - rest_distance/current_distance); // The offset vector that could moves p1 into a distance of rest_distance to p2
		vec3 correctionVectorHalf = correctionVector*0.5f; // Lets make it half that length, so that we can move BOTH p1 and p2.
		p1->offsetPos(correctionVectorHalf); // correctionVectorHalf is pointing from p1 to p2, so the length should move p1 half the length needed to satisfy the constraint.
		p2->offsetPos(-correctionVectorHalf); // we must move p2 the negative direction of correctionVectorHalf since it points from p2 to p1, and not p1 to p2.	
	}

};

class Cloth
{
private:
	struct Vertex {
		vec3 position;
		vec3 color;
		vec3 normal;
	};

	int num_particles_width; // number of particles in "width" direction
	int num_particles_height; // number of particles in "height" direction
	// total number of particles is num_particles_width*num_particles_height

	std::vector<Particle> particles; // all particles that are part of this cloth
	std::vector<Constraint> constraints; // alle constraints between particles as part of this cloth

	Particle* getParticle(int x, int y) {return &particles[y*num_particles_width + x];}
	void makeConstraint(Particle *p1, Particle *p2) {constraints.push_back(Constraint(p1,p2));}


	/* A private method used by drawShaded() and addWindForcesForTriangle() to retrieve the  
	normal vector of the triangle defined by the position of the particles p1, p2, and p3.
	The magnitude of the normal vector is equal to the area of the parallelogram defined by p1, p2 and p3
	*/
	vec3 calcTriangleNormal(Particle *p1,Particle *p2,Particle *p3)
	{
		vec3 pos1 = p1->getPos();
		vec3 pos2 = p2->getPos();
		vec3 pos3 = p3->getPos();

		vec3 v1 = pos2-pos1;
		vec3 v2 = pos3-pos1;

		return cross(v1,v2);
	}

	/* A private method used by windForce() to calcualte the wind force for a single triangle 
	defined by p1,p2,p3*/
	void addWindForcesForTriangle(Particle *p1,Particle *p2,Particle *p3, const vec3 direction)
	{
		vec3 normal = calcTriangleNormal(p1,p2,p3);
		vec3 d = normalize(normal);
		vec3 force = normal*(dot(d,direction));
		p1->addForce(force);
		p2->addForce(force);
		p3->addForce(force);
	}

	/* A private method used by drawShaded(), that draws a single triangle p1,p2,p3 with a color*/
	void drawTriangle(Particle *p1, Particle *p2, Particle *p3, const vec3 color, std::vector<Vertex> &vertexData)
	{
		Vertex v1 = {p1->getPos(), color, p1->getNormal()};
		vertexData.push_back(v1);
		Vertex v2 = {p2->getPos(), color, p2->getNormal()};
		vertexData.push_back(v2);
		Vertex v3 = {p3->getPos(), color, p3->getNormal()};
		vertexData.push_back(v3);
	}

public:

	/* This is a important constructor for the entire system of particles and constraints*/
	Cloth(float width, float height, int num_particles_width, int num_particles_height) : num_particles_width(num_particles_width), num_particles_height(num_particles_height)
	{
		particles.resize(num_particles_width*num_particles_height); //I am essentially using this vector as an array with room for num_particles_width*num_particles_height particles

		// creating particles in a grid of particles from (0,0,0) to (width,-height,0)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				vec3 pos = vec3(width * (x/(float)num_particles_width),
								-height * (y/(float)num_particles_height),
								0);
				particles[y*num_particles_width+x]= Particle(pos); // insert particle in column x at y'th row
			}
		}

		// Connecting immediate neighbor particles with constraints (distance 1 and sqrt(2) in the grid)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-1) makeConstraint(getParticle(x,y),getParticle(x+1,y));
				if (y<num_particles_height-1) makeConstraint(getParticle(x,y),getParticle(x,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeConstraint(getParticle(x,y),getParticle(x+1,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeConstraint(getParticle(x+1,y),getParticle(x,y+1));
			}
		}


		// Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in the grid)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-2) makeConstraint(getParticle(x,y),getParticle(x+2,y));
				if (y<num_particles_height-2) makeConstraint(getParticle(x,y),getParticle(x,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeConstraint(getParticle(x,y),getParticle(x+2,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeConstraint(getParticle(x+2,y),getParticle(x,y+2));			}
		}


		// making the upper left most three and right most three particles unmovable
		for(int i=0;i<3; i++)
		{
			getParticle(0+i ,0)->offsetPos(vec3(0.5,0.0,0.0)); // moving the particle a bit towards the center, to make it hang more natural - because I like it ;)
			getParticle(0+i ,0)->makeUnmovable(); 

			getParticle(0+i ,0)->offsetPos(vec3(-0.5,0.0,0.0)); // moving the particle a bit towards the center, to make it hang more natural - because I like it ;)
			getParticle(num_particles_width-1-i ,0)->makeUnmovable();
		}
	}

	/* drawing the cloth as a smooth shaded (and colored according to column) OpenGL triangular mesh
	Called from the display() method
	The cloth is seen as consisting of triangles for four particles in the grid as follows:

	(x,y)   *--* (x+1,y)
	        | /|
	        |/ |
	(x,y+1) *--* (x+1,y+1)

	*/
	void drawShaded()
	{
		// reset normals (which where written to last frame)
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).resetNormal();
		}

		//create smooth per particle normals by adding up all the (hard) triangle normals that each particle is part of
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				vec3 normal = calcTriangleNormal(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1));
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);

				normal = calcTriangleNormal(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1));
				getParticle(x+1,y+1)->addToNormal(normal);
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);
			}
		}

		static GLuint vertexArrayObject = 0;
		static GLuint vertexBuffer = 0;
		if (vertexArrayObject == 0){
			glGenVertexArrays(1, &vertexArrayObject);
			glBindVertexArray(vertexArrayObject);

			glGenBuffers(1, &vertexBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
			
			GLuint positionAttributeLocation = glGetAttribLocation(litShader, "position");
			GLuint colorAttributeLocation = glGetAttribLocation(litShader, "color");
			GLuint normalAttributeLocation = glGetAttribLocation(litShader, "normal");
			glEnableVertexAttribArray(positionAttributeLocation);
			glEnableVertexAttribArray(colorAttributeLocation);
			glEnableVertexAttribArray(normalAttributeLocation);
			glVertexAttribPointer(positionAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)0);
			glVertexAttribPointer(colorAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)sizeof(vec3));
			glVertexAttribPointer(normalAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)(sizeof(vec3)*2));
		}
		std::vector<Vertex> vertexData;

		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				vec3 color(0,0,0);
				if (x%2) // red and white color is interleaved according to which column number
					color = vec3(0.6f,0.2f,0.2f);
				else
					color = vec3(1.0f,1.0f,1.0f);

				drawTriangle(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1),color, vertexData);
				drawTriangle(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1),color, vertexData);
			}
		}
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(Vertex), value_ptr(vertexData[0].position), GL_STREAM_DRAW);
		mat4 modelView = view; // no model matrix - everything in woldspace
		mat4 mvp = projection * modelView;
		glUniformMatrix4fv(glGetUniformLocation(litShader, "mvp"),1,false, value_ptr(mvp));
		mat3 normalMatrix = inverse(transpose(mat3(modelView)));
		glUniformMatrix3fv(glGetUniformLocation(litShader, "normalMatrix"),1,false, value_ptr(normalMatrix));
	
		glBindVertexArray(vertexArrayObject);
		glDrawArrays(GL_TRIANGLES, 0, vertexData.size());
	}

	/* this is an important methods where the time is progressed one time step for the entire cloth.
	This includes calling satisfyConstraint() for every constraint, and calling timeStep() for all particles
	*/
	void timeStep()
	{
		std::vector<Constraint>::iterator constraint;
		for(int i=0; i<CONSTRAINT_ITERATIONS; i++) // iterate over all constraints several times
		{
			for(constraint = constraints.begin(); constraint != constraints.end(); constraint++ )
			{
				(*constraint).satisfyConstraint(); // satisfy constraint.
			}
		}

		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).timeStep(); // calculate the position of each particle at the next time step.
		}
	}

	/* used to add gravity (or any other arbitrary vector) to all particles*/
	void addForce(const vec3 direction)
	{
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).addForce(direction); // add the forces to each particle
		}

	}

	/* used to add wind forces to all particles, is added for each triangle since the final force is proportional to the triangle area as seen from the wind direction*/
	void windForce(const vec3 direction)
	{
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				addWindForcesForTriangle(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1),direction);
				addWindForcesForTriangle(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1),direction);
			}
		}
	}

	/* used to detect and resolve the collision of the cloth with the ball.
	This is based on a very simples scheme where the position of each particle is simply compared to the sphere and corrected.
	This also means that the sphere can "slip through" if the ball is small enough compared to the distance in the grid bewteen particles
	*/
	void ballCollision(const vec3 center,const float radius )
	{
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			vec3 v = (*particle).getPos()-center;
			float l = length(v);
			if ( length(v) < radius) // if the particle is inside the ball
			{
				(*particle).offsetPos(normalize(v)*(radius-l)); // project the particle to the surface of the ball
			}
		}
	}

	void doFrame()
	{

	}
};

/***** Above are definition of classes; vec3, Particle, Constraint, and Cloth *****/




// Just below are three global variables holding the actual animated stuff; Cloth and Ball 
Cloth cloth1(14,10,55,45); // one Cloth object of the Cloth class
vec3 ball_pos(7,-5,0); // the center of our one ball
float ball_radius = 2; // the radius of our one ball



/***** Below are functions Init(), display(), reshape(), keyboard(), arrow_keys(), main() *****/

/* This is where all the standard Glut/OpenGL stuff is, and where the methods of Cloth are called; 
addForce(), windForce(), timeStep(), ballCollision(), and drawShaded()*/

void init()
{
    reshape(w,h);
    glViewport(0,0,w,h);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);				
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	
	lightPos0 = vec4(-1.0,1.0,0.5,0.0);
	vec4 lightAmbient0 = vec4(0.2, 0.2, 0.2, 1.0);
	vec4 lightDiffuse0 = vec4(0.8, 0.8, 0.8, 1.0);
	
	lightPos1 = vec4(1.0,0.0,-0.2,0.0);
	vec4 lightAmbient1 = vec4(0.0,0.0,0.0,0.0);
	vec4 lightDiffuse1 = vec4(0.5,0.5,0.3,0.0);

	vec4 ambient[2] = {lightAmbient0, lightAmbient1};
	vec4 diffuse[2] = {lightDiffuse0, lightDiffuse1};
	glUseProgram(litShader);
	glUniform4fv(glGetUniformLocation(litShader, "lightAmbient"),2, value_ptr(ambient[0]));
	glUniform4fv(glGetUniformLocation(litShader, "lightDiffuse"),2, value_ptr(diffuse[0]));

	vec4 lightModelAmbient = vec4(0.2, 0.2, 0.2, 1.0);
	glUniform4fv(glGetUniformLocation(litShader, "lightModelAmbient"),1, value_ptr(lightModelAmbient));
}


float ball_time = 0; // counter for used to calculate the z position of the ball below

void drawSolidSphere(vec3& position){
	glUseProgram(litShader);
	static GLuint vertexArrayObject = 0;
	static int elementCount;
	if (vertexArrayObject == 0){
		struct Vertex {
			vec3 position;
			vec3 color;
			vec3 normal;
		};
		vec3 color = vec3(0.4f,0.8f,0.5f);
		std::vector<Vertex> vertexData;
		int slices = 64;
        int stacks = 32;
        float radius = 1.9;
        int vertexCount = (stacks+1) * (slices + 1);
        float piDivStacks = M_PI / stacks;
        float PIDiv2 = M_PI / 2;
        float PI2 = M_PI * 2;
        
		for (int j = 0; j <= stacks; j++) {
			float latitude1 = piDivStacks * j - PIDiv2;
            float sinLat1 = sin(latitude1);
            float cosLat1 = cos(latitude1);
            for (int i = 0; i <= slices; i++) {
                float longitude = (PI2 / slices) * i;
                float sinLong = sin(longitude);
                float cosLong = cos(longitude);
				vec3 normal = vec3(cosLong * cosLat1, sinLat1, sinLong * cosLat1);
				vec3 position = normal * radius;
				Vertex v = {position, color, normal};
				vertexData.push_back(v);
            }
        }
		std::vector<GLuint> indices;
        // create indices
        for (int j = 0; j < stacks; j++) {
			int index;
            if (j > 0) {
                indices.push_back(j * (slices + 1)); // make degenerate
            }
            for (int i = 0; i <= slices; i++) {
                index = j * (slices + 1) + i;
                indices.push_back(index);
                indices.push_back(index + slices + 1);
            }
            if (j + 1 < stacks) {
                indices.push_back(index + slices + 1); // make degenerate
            }
        }
		
		glGenVertexArrays(1, &vertexArrayObject);
		glBindVertexArray(vertexArrayObject);

		GLuint vertexBuffer;
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(Vertex), value_ptr(vertexData[0].position), GL_STATIC_DRAW);

		GLuint positionAttributeLocation = glGetAttribLocation(litShader, "position");
		GLuint colorAttributeLocation = glGetAttribLocation(litShader, "color");
		GLuint normalAttributeLocation = glGetAttribLocation(litShader, "normal");
		glEnableVertexAttribArray(positionAttributeLocation);
		glEnableVertexAttribArray(colorAttributeLocation);
		glEnableVertexAttribArray(normalAttributeLocation);
		glVertexAttribPointer(positionAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)0);
		glVertexAttribPointer(colorAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)sizeof(vec3));
		glVertexAttribPointer(normalAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)(sizeof(vec3)*2));

		GLuint elementArrayBuffer;
		glGenBuffers(1, &elementArrayBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementArrayBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), &(indices[0]), GL_STATIC_DRAW);
		elementCount = indices.size();

	}
	
	mat4 modelView = view;
	modelView = translate(modelView, position);
	mat4 mvp = projection * modelView;
	glUniformMatrix4fv(glGetUniformLocation(litShader, "mvp"),1,false, value_ptr(mvp));
	mat3 normalMatrix = inverse(transpose(mat3(modelView)));
	glUniformMatrix3fv(glGetUniformLocation(litShader, "normalMatrix"),1,false, value_ptr(normalMatrix));
	
	glBindVertexArray(vertexArrayObject);
	glDrawElements(GL_TRIANGLE_STRIP, elementCount, GL_UNSIGNED_INT, 0);
}

void drawScreenQuad() {
	glUseProgram(unlitShader);
	static GLuint vertexArrayObject = 0;
	if (vertexArrayObject == 0){
		struct Vertex {
			vec3 position;
			vec3 color;
		};
		Vertex vertexData[4] = {
			{vec3(-200.0f,-100.0f,-100.0f ), vec3( 0.8f,0.8f,1.0f )},
			{vec3( 200.0f,-100.0f,-100.0f ), vec3( 0.8f,0.8f,1.0f )}, 
			{vec3(-200.0f, 100.0f,-100.0f ), vec3( 0.4f,0.4f,0.8f )},
			{vec3( 200.0f, 100.0f,-100.0f ), vec3( 0.4f,0.4f,0.8f )}, 
		};

		glGenVertexArrays(1, &vertexArrayObject);
		glBindVertexArray(vertexArrayObject);

		GLuint vertexBuffer;
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(Vertex), value_ptr(vertexData[0].position), GL_STATIC_DRAW);

		GLuint positionAttributeLocation = glGetAttribLocation(unlitShader, "position");
		GLuint colorAttributeLocation = glGetAttribLocation(unlitShader, "color");
		glEnableVertexAttribArray(positionAttributeLocation);
		glEnableVertexAttribArray(colorAttributeLocation);
		glVertexAttribPointer(positionAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)0);
		glVertexAttribPointer(colorAttributeLocation, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid *)sizeof(vec3));
	}
	glUniformMatrix4fv(glGetUniformLocation(unlitShader, "mvp"),1,false, value_ptr(projection));
	glBindVertexArray(vertexArrayObject);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

/* display method called each frame*/
void display(void)
{
	// calculating positions
    ball_time++;
	ball_pos[2] = (float)cos(ball_time/50.0f)*7;

	cloth1.addForce(vec3(0,-0.2,0)*TIME_STEPSIZE2); // add gravity each frame, pointing down
	cloth1.windForce(vec3(0.5,0,0.2)*TIME_STEPSIZE2); // generate some wind each frame
	cloth1.timeStep(); // calculate the particle positions of the next frame
	cloth1.ballCollision(ball_pos,ball_radius); // resolve collision with the ball

	// drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    view = mat4(1.0f);
	drawScreenQuad(); // drawing some smooth shaded background - because I like it ;)

    view = translate(view, vec3(-6.5, 6, -9.0f));
    view = rotate(view, glm::radians(25.0f), vec3(0,1,0));

	// setup light 
	glUseProgram(litShader);
	vec4 eyeSpaceLight[2] = { lightPos0, lightPos1};
	glUniform4fv(glGetUniformLocation(litShader, "lightPosition"), 2, value_ptr(eyeSpaceLight[0]));
	cloth1.drawShaded();

	drawSolidSphere(ball_pos);

    SDL_GL_SwapWindow(mainwindow);
    SDL_Delay(16);
}

void reshape(int w, int h)  
{
	if (h==0)
		projection = perspective(glm::radians(80.0f),(float)w,1.0f,5000.0f);
	else
		projection = perspective(glm::radians(80.0f),( float )w /( float )h, 1.0f, 5000.0f);
}

void keyboard( unsigned char key, int x, int y ) 
{
	switch ( key ) {
	case 27:    
		exit ( 0 );
		break;  
	default: 
		break;
	}
}

void arrow_keys( int a_keys, int x, int y ) 
{
	switch(a_keys) {
	case GLUT_KEY_UP:
		glutFullScreen();
		break;
	case GLUT_KEY_DOWN: 
		glutReshapeWindow (1280, 720 );
		break;
	default:
		break;
	}
}

void checkCompileStatus(GLuint shader, const char *shadername){
	GLint  compiled;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &compiled );
	if ( !compiled ) {
		std::cerr << shadername << " failed to compile:" << std::endl;
		GLint  logSize;
		glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &logSize );
		char* logMsg = new char[logSize];
		glGetShaderInfoLog( shader, logSize, NULL, logMsg );
		std::cerr << logMsg << std::endl;
		delete [] logMsg;
	}
}

void checkLinkStatus(GLuint program, const char * programName){
	GLint  linked;
	glGetProgramiv( program, GL_LINK_STATUS, &linked );
	if ( !linked ) {
		std::cerr << "Shader program " << programName << " failed to link" << std::endl;
		GLint  logSize;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &logSize);
		char* logMsg = new char[logSize];
		glGetProgramInfoLog( program, logSize, NULL, logMsg );
		std::cerr << logMsg << std::endl;
		delete [] logMsg;
	}
}

GLuint loadShader(const char* vertexShaderName, const char* fragmentShaderName){
	const char *vertexShaderSource = TextResource::load(vertexShaderName);
	GLuint vertexShader = glCreateShader( GL_VERTEX_SHADER );
	glShaderSource( vertexShader, 1, &vertexShaderSource, NULL );
	glCompileShader( vertexShader );
	checkCompileStatus( vertexShader, vertexShaderName );

	const char *fragmentShaderSource = TextResource::load(fragmentShaderName);
	GLuint fragmentShader = glCreateShader( GL_FRAGMENT_SHADER );
	glShaderSource( fragmentShader, 1, &fragmentShaderSource, NULL );
	glCompileShader( fragmentShader );
	checkCompileStatus( fragmentShader, fragmentShaderName );

	GLuint program = glCreateProgram();
	glAttachShader( program, vertexShader );
	glAttachShader( program, fragmentShader );
	glLinkProgram(program);
	checkLinkStatus(program, vertexShaderName);
	return program;
}

void renderloop(){

    /*glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(arrow_keys);

    glutMainLoop();
     */
    while (true){
        SDL_GetWindowSize(mainwindow,&w,&h);
        reshape(w,h);
        display();

    }
}

int main(int &argc, char** argv)
{
    SDL_GLContext maincontext; /* Our opengl context handle */

    if (SDL_Init(SDL_INIT_VIDEO) < 0) /* Initialize SDL's Video subsystem */
        std::cout<<("Unable to initialize SDL"); /* Or die on error */

    /* Request opengl 3.2 context.
     * SDL doesn't have the ability to choose which profile at this time of writing,
     * but it should default to the core profile */
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    w = 1280;
    h = 720;
    /* Create our window centered at 512x512 resolution */
    mainwindow = SDL_CreateWindow("Cloth Tutorial Refactoring OpenGL 3.2 Core profile" , SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                  w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!mainwindow) /* Die if creation failed */
        std::cout<<("Unable to create window");


    /* Create our opengl context and attach it to our window */
    maincontext = SDL_GL_CreateContext(mainwindow);
    SDL_GL_SetSwapInterval(1);


#ifdef _WIN32
	glewExperimental = true;
	GLint GlewInitResult = glewInit();
	if (GlewInitResult != GLEW_OK) {
		printf("ERROR: %s\n", glewGetErrorString(GlewInitResult));
	}
#endif
	litShader = loadShader("cloth_3_2_core_profile/lambert.vert", "cloth_3_2_core_profile/lambert.frag");
	unlitShader = loadShader("cloth_3_2_core_profile/unlit.vert", "cloth_3_2_core_profile/unlit.frag");

	init();

    renderloop();

    SDL_GL_DeleteContext(maincontext);
    SDL_DestroyWindow(mainwindow);
    SDL_Quit();

	return 0;
}

}
