#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (SplitMethod::NAIVE)
        {
        case SplitMethod::NAIVE:
            {
            switch (dim) {
			case 0:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
					});
				break;
			case 1:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
					});
				break;
			case 2:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
					});
				break;
			}

			auto beginning = objects.begin();
			auto middling = objects.begin() + (objects.size() / 2);
			auto ending = objects.end();

			auto leftshapes = std::vector<Object*>(beginning, middling);
			auto rightshapes = std::vector<Object*>(middling, ending);

			assert(objects.size() == (leftshapes.size() + rightshapes.size()));

			node->left = recursiveBuild(leftshapes);
			node->right = recursiveBuild(rightshapes);

			node->bounds = Union(node->left->bounds, node->right->bounds);
            }
            break;

        case SplitMethod::SAH:
            {
			float SN = centroidBounds.SurfaceArea();
			int B = 10;
			int mincostIndex = 0;
			float minCost = std::numeric_limits<float>::infinity();
			switch (dim) {
			case 0:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
					});
				break;
			case 1:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
					});
				break;
			case 2:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
					});
			}

			for (int i = 1; i < B; i++)
			{
				auto beginning = objects.begin();
				auto middling = objects.begin() + (objects.size() * i / B);
				auto end = objects.end();
				auto leftShapes = std::vector<Object*>(beginning, middling);
				auto rightShapes = std::vector<Object*>(middling, end);

				Bounds3 leftBounds, rightBounds;
				for (int k = 0; i < leftShapes.size(); k++)
				{
					leftBounds = Union(leftBounds, leftShapes[k]->getBounds().Centroid());
				}
				for (int k = 0; k < rightShapes.size(); k++)
				{
					rightBounds = Union(rightBounds, rightShapes[i]->getBounds().Centroid());

				}

				float SA = leftBounds.SurfaceArea();
				float SB = rightBounds.SurfaceArea();

				float cost = 0.125f + (leftShapes.size() * SA + rightShapes.size() * SB) / SN;
				if (cost < minCost)
				{
					minCost = cost;
					mincostIndex = i;
				}

				auto beginning1 = objects.begin();
				auto middling1 = objects.begin() + (objects.size() * mincostIndex / B);
				auto ending1 = objects.end();
				auto leftShapes1 = std::vector<Object*>(beginning1, middling1);
				auto rightShapes1 = std::vector<Object*>(middling1, ending1);

				assert(objects.size() == leftShapes1.size() + rightShapes1.size());

				node->left = recursiveBuild(leftShapes);
				node->right = recursiveBuild(rightShapes);
				node->bounds = Union(node->left->bounds, node->right->bounds);

			}

            }
        }

		return node;
    }
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{

    // TODO Traverse the BVH to find intersection
	Intersection hitInfo;
	std::array<int, 3> sign;
	sign[0] = ray.direction.x > 0;
	sign[1] = ray.direction.y > 0;
	sign[2] = ray.direction.z > 0;
	bool isHit = node->bounds.IntersectP(ray, ray.direction_inv, sign);
	if (!node || !isHit)
	{
		return hitInfo;
	}

	if (!(node->left) && !(node->right))
	{
		return node->object->getIntersection(ray);
	}

	Intersection hit1Info = getIntersection(node->left, ray);
	Intersection hit2Info = getIntersection(node->right, ray);

	return hit1Info.distance > hit2Info.distance ? hit2Info : hit1Info;

}