// Compile to mex file with
// >> mex mexVHACD\mexVHACD.cpp
// and add option -g if you need debug

#include "mex.hpp"
#include "mexAdapter.hpp"

using namespace matlab::data;
using matlab::mex::ArgumentList;

#include <iostream>

#define ENABLE_VHACD_IMPLEMENTATION 1
#define VHACD_DISABLE_THREADING 0
#include "VHACD.h"

class MexFunction : public matlab::mex::Function
{
	matlab::data::ArrayFactory factory;

public:
	void operator()(ArgumentList outputs, ArgumentList inputs)
	{
        /*
        * Inputs are point positions (nV * 3 double array),
        * IDs of triangles' vertices and max number of convex hulls.
        * Outputs are the generated convex hulls.
        */

		// Validate arguments
		checkArguments(outputs, inputs);

		// Translate variables

        // points (inputs[0])
		uint32_t countPoints = inputs[0].getDimensions()[0];
		double* points = new double[countPoints * 3];
		for (int i = 0; i < countPoints * 3; i++)
			points[i] = inputs[0][i / 3][i % 3];
        // triangles (inputs[1])
		uint32_t countTriangles = inputs[1].getDimensions()[0];
		uint32_t* triangles = new uint32_t[countTriangles * 3];
		for (int i = 0; i < countTriangles * 3; i++)
			triangles[i] = inputs[1][i / 3][i % 3];
        // maxConvexHulls (inputs[2])
        uint32_t maxConvexHulls = inputs[2][0];

		// Implement function
        VHACD::IVHACD::Parameters p;
        p.m_maxConvexHulls = maxConvexHulls;
        VHACD::IVHACD* iface = VHACD::CreateVHACD();
		iface->Compute(points, countPoints, triangles, countTriangles, p);

		// Assign outputs
		int nConvHull = iface->GetNConvexHulls();
		TypedArray<uint32_t> offsetPoints = factory.createArray<uint32_t>({ 1, 1 + iface->GetNConvexHulls() });
		TypedArray<uint32_t> offsetTriangles = factory.createArray<uint32_t>({ 1, 1 + iface->GetNConvexHulls() });
		offsetPoints[0] = 0;
		offsetTriangles[0] = 0;

		for (int i = 0; i < iface->GetNConvexHulls(); i++)
		{
			VHACD::IVHACD::ConvexHull ch;
			iface->GetConvexHull(i, ch);
			offsetPoints[i + 1] = offsetPoints[i] + ch.m_points.size();
			offsetTriangles[i + 1] = offsetTriangles[i] + ch.m_triangles.size();
		}

		TypedArray<double> posPoints = factory.createArray<double>({ offsetPoints[nConvHull], 3 });
		TypedArray<int> vidTriangles = factory.createArray<int>({ offsetTriangles[nConvHull], 3 });
		for (int i = 0; i < iface->GetNConvexHulls(); i++)
		{
			VHACD::IVHACD::ConvexHull ch;
			iface->GetConvexHull(i, ch);

			for (int j = 0; j < ch.m_points.size(); j++)
			{
				const VHACD::Vertex& pos = ch.m_points[j];
				posPoints[j + offsetPoints[i]][0] = pos.mX;
				posPoints[j + offsetPoints[i]][1] = pos.mY;
				posPoints[j + offsetPoints[i]][2] = pos.mZ;
			}

			for (int j = 0; j < ch.m_triangles.size(); j++)
			{
				uint32_t i0 = ch.m_triangles[j].mI0 + 1;
				uint32_t i1 = ch.m_triangles[j].mI1 + 1;
				uint32_t i2 = ch.m_triangles[j].mI2 + 1;
				vidTriangles[j + offsetTriangles[i]][0] = i0;
				vidTriangles[j + offsetTriangles[i]][1] = i1;
				vidTriangles[j + offsetTriangles[i]][2] = i2;
			}
		}

		outputs[0] = std::move(posPoints);
		outputs[1] = std::move(offsetPoints);
		outputs[2] = std::move(vidTriangles);
		outputs[3] = std::move(offsetTriangles);
	}

	void checkArguments(ArgumentList outputs, ArgumentList inputs)
	{
		// Get pointer to engine
		std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

		// Check number of inputs
		if (inputs.size() != 3)
		{
			matlabPtr->feval(u"error",
				0,
				std::vector<matlab::data::Array>({ factory.createScalar("Three inputs required") }));
		}

		// Check number of outputs
		if (outputs.size() > 4)
		{
			matlabPtr->feval(u"error",
				0,
				std::vector<Array>({ factory.createScalar("Only Four outputs are returned") }));
		}

		// Check array argument: First input must be double array
		if (inputs[0].getType() != ArrayType::DOUBLE ||
			inputs[0].getType() == ArrayType::COMPLEX_DOUBLE)
		{
			matlabPtr->feval(u"error",
				0,
				std::vector<Array>({ factory.createScalar("First Input (points) must be double array") }));
		}

		// Check array argument: First input must be double array
		if (inputs[1].getType() != ArrayType::DOUBLE)
		{
			matlabPtr->feval(u"error",
				0,
				std::vector<Array>({ factory.createScalar("Second Input (points) must be int array") }));
		}
	}
};