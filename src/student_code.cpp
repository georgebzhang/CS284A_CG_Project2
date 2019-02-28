#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
	Vector2D lerp2D(Vector2D p0, Vector2D p1, float t) {
		double xi = (1 - t) * p0.x + t * p1.x;
		double yi = (1 - t) * p0.y + t * p1.y;
		return Vector2D(xi, yi);
	}

	void BezierCurve::evaluateStep() // called in bezierCurve.cpp in render(...) and n-1 times in drawCurve(...) where n is the number of control points
	{
		// TODO Part 1.
		// Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
		// Store all of the intermediate control points into the 2D vector evaluatedLevels.

		std::vector<Vector2D> intermediateControlPoints;
		int numLevels = evaluatedLevels.size();
		for (int i = 0; i < evaluatedLevels[numLevels - 1].size() - 1; ++i) {
			intermediateControlPoints.push_back(lerp2D(evaluatedLevels[numLevels - 1][i], evaluatedLevels[numLevels - 1][i+1], t));
		}
		evaluatedLevels.push_back(intermediateControlPoints);
		return;
	}

	Vector3D lerp3D(Vector3D p0, Vector3D p1, float t) {
		double xi = (1 - t) * p0.x + t * p1.x;
		double yi = (1 - t) * p0.y + t * p1.y;
		double zi = (1 - t) * p0.z + t * p1.z;
		return Vector3D(xi, yi, zi);
	}

	Vector3D BezierPatch::evaluate(double u, double v) const
	{
		// TODO Part 2.
		// Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
		// (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
		// should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)

		std::vector<Vector3D> uPoints;
		for (int i = 0; i < controlPoints.size(); ++i) {
			uPoints.push_back(evaluate1D(controlPoints[i], u));
		}
		return evaluate1D(uPoints, v);
	}

	Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
	{
		// TODO Part 2.
		// Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
		// Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

		std::vector<std::vector<Vector3D>> evaluatedLevels;
		std::vector<Vector3D> intermediatePoints;
		evaluatedLevels.push_back(points);
		for (int i = 0; i < points.size() - 1; ++i) {
			for (int j = 0; j < evaluatedLevels[i].size() - 1; ++j) {
				intermediatePoints.push_back(lerp3D(evaluatedLevels[i][j], evaluatedLevels[i][j + 1], t));
				
			}
			evaluatedLevels.push_back(intermediatePoints);
			intermediatePoints.clear();
			
		}
		return evaluatedLevels[points.size()-1][0];
	}

	/**
	* Returns average of vectors.
	*/
	Vector3D average(std::vector<Vector3D>& vectors) {
		double x_total = 0, y_total = 0, z_total = 0;
		double x_avg, y_avg, z_avg;
		int sz = vectors.size();
		for (int i = 0; i < sz; ++i) {
			x_total += vectors[i].x;
			y_total += vectors[i].y;
			z_total += vectors[i].z;
		}
		x_avg = x_total / sz;
		y_avg = y_total / sz;
		z_avg = z_total / sz;
		return Vector3D(x_avg, y_avg, z_avg);
	}

	Vector3D Vertex::normal( void ) const
	{
		// TODO Part 3.
		// TODO Returns an approximate unit normal at this vertex, computed by
		// TODO taking the area-weighted average of the normals of neighboring
		// TODO triangles, then normalizing.

		std::vector<Vector3D> normals;

		int count = 0; // start calculating normals and areas after the first iteration
		HalfedgeCIter h = halfedge();
		HalfedgeCIter h_orig = h;
		VertexCIter v_orig = h->vertex();
		Vector3D pos_orig = v_orig->position;
		VertexCIter v_prev;

		do {
			HalfedgeCIter h_twin = h->twin();
			VertexCIter v = h_twin->vertex();
			if (count > 0) {
				Vector3D edge1 = v_prev->position - pos_orig;
				Vector3D edge2 = v->position - pos_orig;
				Vector3D normal = -cross(edge1, edge2);
				normals.push_back(normal);
			}
			h = h_twin->next();
			v_prev = v;
			++count;
		} while (h != h_orig);

		Vector3D normal_avg = average(normals);
		normal_avg = normal_avg.unit();
		return normal_avg;
	}

	EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
	{
		// TODO Part 4.
		// TODO This method should flip the given edge and return an iterator to the flipped edge.

		if (e0->isBoundary()) {
			std::cout << "Trying to flip boundary edge!" << std::endl;
			return e0;
		}

		// collect halfedges
		HalfedgeIter h0 = e0->halfedge();
		HalfedgeIter h1 = h0->next();
		HalfedgeIter h2 = h1->next();
		HalfedgeIter h3 = h0->twin();
		HalfedgeIter h4 = h3->next();
		HalfedgeIter h5 = h4->next();
		HalfedgeIter h6 = h1->twin();
		HalfedgeIter h7 = h2->twin();
		HalfedgeIter h8 = h4->twin();
		HalfedgeIter h9 = h5->twin();
		// collect vertices
		VertexIter v0 = h0->vertex();
		VertexIter v1 = h3->vertex();
		VertexIter v2 = h2->vertex();
		VertexIter v3 = h5->vertex();
		// collect edges
		EdgeIter e1 = h1->edge();
		EdgeIter e2 = h2->edge();
		EdgeIter e3 = h4->edge();
		EdgeIter e4 = h5->edge();
		// collect faces
		FaceIter f0 = h0->face();
		FaceIter f1 = h3->face();

		// assign halfedges
		h0->next() = h1;
		h0->twin() = h3;
		h0->vertex() = v3;
		h0->edge() = e0;
		h0->face() = f0;

		h1->next() = h2;
		h1->twin() = h7;
		h1->vertex() = v2;
		h1->edge() = e2;
		h1->face() = f0;

		h2->next() = h0;
		h2->twin() = h8;
		h2->vertex() = v0;
		h2->edge() = e3;
		h2->face() = f0;

		h3->next() = h4;
		h3->twin() = h0;
		h3->vertex() = v2;
		h3->edge() = e0;
		h3->face() = f1;

		h4->next() = h5;
		h4->twin() = h9;
		h4->vertex() = v3;
		h4->edge() = e4;
		h4->face() = f1;

		h5->next() = h3;
		h5->twin() = h6;
		h5->vertex() = v1;
		h5->edge() = e1;
		h5->face() = f1;

		//h6->next() = h6->next();
		h6->twin() = h5;
		h6->vertex() = v2;
		h6->edge() = e1;
		//h6->face() = h6->face();

		//h7->next() = h7->next();
		h7->twin() = h1;
		h7->vertex() = v0;
		h7->edge() = e2;
		//h7->face() = h7->face();

		//h8->next() = h8->next();
		h8->twin() = h2;
		h8->vertex() = v3;
		h8->edge() = e3;
		//h8->face() = h8->face();

		//h9->next() = h9->next();
		h9->twin() = h4;
		h9->vertex() = v1;
		h9->edge() = e4;
		//h9->face() = h9->face();

		// assign vertices
		v0->halfedge() = h2;
		v1->halfedge() = h5;
		v2->halfedge() = h3;
		v3->halfedge() = h0;

		// assign edges
		e0->halfedge() = h0;
		e1->halfedge() = h5;
		e2->halfedge() = h1;
		e3->halfedge() = h2;
		e4->halfedge() = h4;

		// assign faces
		f0->halfedge() = h0;
		f1->halfedge() = h3;

		return e0;
	}

	VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
	{
		// TODO Part 5.
		// TODO This method should split the given edge and return an iterator to the newly inserted vertex.
		// TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

		if (e0->isBoundary()) {
			std::cout << "Trying to flip boundary edge!" << std::endl;
			return newVertex();
		}

		// collect halfedges
		HalfedgeIter h0 = e0->halfedge();
		HalfedgeIter h1 = h0->next();
		HalfedgeIter h2 = h1->next();
		HalfedgeIter h3 = h0->twin();
		HalfedgeIter h4 = h3->next();
		HalfedgeIter h5 = h4->next();
		HalfedgeIter h6 = h1->twin();
		HalfedgeIter h7 = h2->twin();
		HalfedgeIter h8 = h4->twin();
		HalfedgeIter h9 = h5->twin();
		// collect vertices
		VertexIter v0 = h0->vertex();
		VertexIter v1 = h3->vertex();
		VertexIter v2 = h2->vertex();
		VertexIter v3 = h5->vertex();
		// collect edges
		EdgeIter e1 = h1->edge();
		EdgeIter e2 = h2->edge();
		EdgeIter e3 = h4->edge();
		EdgeIter e4 = h5->edge();
		// collect faces
		FaceIter f0 = h0->face();
		FaceIter f1 = h3->face();

		// create mew halfedges
		HalfedgeIter h10 = newHalfedge();
		HalfedgeIter h11 = newHalfedge();
		HalfedgeIter h12 = newHalfedge();
		HalfedgeIter h13 = newHalfedge();
		HalfedgeIter h14 = newHalfedge();
		HalfedgeIter h15 = newHalfedge();
		// create new vertices
		VertexIter v4 = newVertex();
		std::vector<Vector3D> positions;
		positions.push_back(v0->position);
		positions.push_back(v1->position);
		v4->position = average(positions);
		// create new edges
		EdgeIter e5 = newEdge();
		EdgeIter e6 = newEdge();
		EdgeIter e7 = newEdge();
		// create new faces
		FaceIter f2 = newFace();
		FaceIter f3 = newFace();

		// assign halfedges
		h0->next() = h1;
		h0->twin() = h3;
		h0->vertex() = v4;
		h0->edge() = e0;
		h0->face() = f0;

		h1->next() = h12;
		h1->twin() = h6;
		h1->vertex() = v1;
		h1->edge() = e1;
		h1->face() = f0;

		h2->next() = h15;
		h2->twin() = h7;
		h2->vertex() = v2;
		h2->edge() = e2;
		h2->face() = f3;

		h3->next() = h10;
		h3->twin() = h0;
		h3->vertex() = v1;
		h3->edge() = e0;
		h3->face() = f1;

		h4->next() = h11;
		h4->twin() = h8;
		h4->vertex() = v0;
		h4->edge() = e3;
		h4->face() = f2;

		h5->next() = h3;
		h5->twin() = h9;
		h5->vertex() = v3;
		h5->edge() = e4;
		h5->face() = f1;

		//h6->next() = h6->next();
		h6->twin() = h1;
		h6->vertex() = v2;
		h6->edge() = e1;
		//h6->face() = h6->face();

		//h7->next() = h7->next();
		h7->twin() = h2;
		h7->vertex() = v0;
		h7->edge() = e2;
		//h7->face() = h7->face();

		//h8->next() = h8->next();
		h8->twin() = h4;
		h8->vertex() = v3;
		h8->edge() = e3;
		//h8->face() = h8->face();

		//h9->next() = h9->next();
		h9->twin() = h5;
		h9->vertex() = v1;
		h9->edge() = e4;
		//h9->face() = h9->face();

		h10->next() = h5;
		h10->twin() = h11;
		h10->vertex() = v4;
		h10->edge() = e6;
		h10->face() = f1;

		h11->next() = h14;
		h11->twin() = h10;
		h11->vertex() = v3;
		h11->edge() = e6;
		h11->face() = f2;

		h12->next() = h0;
		h12->twin() = h13;
		h12->vertex() = v2;
		h12->edge() = e7;
		h12->face() = f0;

		h13->next() = h2;
		h13->twin() = h12;
		h13->vertex() = v4;
		h13->edge() = e7;
		h13->face() = f3;

		h14->next() = h4;
		h14->twin() = h15;
		h14->vertex() = v4;
		h14->edge() = e5;
		h14->face() = f2;

		h15->next() = h13;
		h15->twin() = h14;
		h15->vertex() = v0;
		h15->edge() = e5;
		h15->face() = f3;

		// assign vertices
		v0->halfedge() = h15;
		v1->halfedge() = h3;
		v2->halfedge() = h12;
		v3->halfedge() = h11;
		v4->halfedge() = h0;

		// assign edges
		e0->halfedge() = h0;
		e1->halfedge() = h1;
		e2->halfedge() = h2;
		e3->halfedge() = h4;
		e4->halfedge() = h5;
		e5->halfedge() = h14;
		e6->halfedge() = h10;
		e7->halfedge() = h13;

		// assign faces
		f0->halfedge() = h0;
		f1->halfedge() = h3;
		f2->halfedge() = h14;
		f3->halfedge() = h15;

		return v4;
	}



	void MeshResampler::upsample( HalfedgeMesh& mesh ) // class MeshResampler in student_code.h
	{
		// TODO Part 6.
		// This routine should increase the number of triangles in the mesh using Loop subdivision.
		// Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
		// Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
		// using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
		// the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
		// the new mesh based on the values we computed for the original mesh.

		// TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
		// TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
		// TODO a vertex of the original mesh.
		for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
			int n = v->degree();
			double u = n == 3 ? (double) 3/ (double) 16 : (double) 3/((double) 8 * (double) n);
			Vector3D neighbor_position_sum = 0; // or v->position?
			HalfedgeCIter h = v->halfedge();
			HalfedgeCIter h_orig = h;
			do {
				HalfedgeCIter h_twin = h->twin();
				VertexCIter v = h_twin->vertex();
				neighbor_position_sum += v->position;
				h = h_twin->next();
			} while (h != h_orig);
			v->newPosition = (1 - (double) n*u) * v->position + u*neighbor_position_sum;
			v->isNew = false;
		}

		// TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
		for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
			std::vector<Vector3D> positions;
			HalfedgeCIter h = e->halfedge();
			VertexCIter vA = h->vertex();
			VertexCIter vB = h->twin()->vertex();
			VertexCIter vC = h->next()->next()->vertex();
			VertexCIter vD = h->twin()->next()->next()->vertex();
			Vector3D pA = vA->position;
			Vector3D pB = vB->position;
			Vector3D pC = vC->position;
			Vector3D pD = vD->position;
			e->newPosition = (double) 3/ (double) 8 * (pA + pB) + (double) 1/ (double) 8 * (pC + pD);
			e->isNew = false;
		}

		// TODO Next, we're going to split every edge in the mesh, in any order.  For future
		// TODO reference, we're also going to store some information about which subdivided
		// TODO edges come from splitting an edge in the original mesh, and which edges are new,
		// TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
		// TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
		// TODO just split (and the loop will never end!)
		std::vector<EdgeIter> new_edges;
		for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
			if (!e->isNew) {
				VertexIter v = mesh.splitEdge(e);
				v->position = e->newPosition;
				v->isNew = true;
				HalfedgeIter h = v->halfedge();
				EdgeIter e0 = h->edge();
				EdgeIter e1 = h->twin()->next()->twin()->next()->edge();
				EdgeIter e2 = h->next()->next()->edge();
				EdgeIter e3 = h->twin()->next()->edge();
				e0->isNew = true;
				e1->isNew = true;
				e2->isNew = true;
				e3->isNew = true;
				new_edges.push_back(e2);
				new_edges.push_back(e3);
			}
		}

		// TODO Now flip any new edge that connects an old and new vertex.
		for (int i = 0; i < new_edges.size(); ++i) {
			HalfedgeCIter h = new_edges[i]->halfedge();
			VertexCIter v0 = h->vertex();
			VertexCIter v1 = h->twin()->vertex();
			if (v0->isNew != v1->isNew)
				mesh.flipEdge(new_edges[i]);
		}

		// TODO Finally, copy the new vertex positions into final Vertex::position.
		for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
			if (!v->isNew)
				v->position = v->newPosition;
		}

		return;
	}
}
