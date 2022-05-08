//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }

    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

	//intersect(const Ray ray)in Scene.cpp: 求一条光线与场景的交点
	//	• sampleLight(Intersection pos, float pdf) in Scene.cpp : 在场景的所有
	//	光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度
	//	3
	//	• sample(const Vector3f wi, const Vector3f N) in Material.cpp : 按照该
	//	材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
	//	• pdf(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp: 给定一对入射、出射方向与法向量，计算 sample 方法得到该出射
	//	方向的概率密度
	//	• eval(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp: 给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
	//	可能用到的变量有：
	//	• RussianRoulette in Scene.cpp : P_RR, Russian Roulette 的概率

	//if (depth > 2) return Vector3f();

	Intersection hitPoint = intersect(ray);

	if (!hitPoint.happened)
		return Vector3f();

	if (hitPoint.m->hasEmission())
		return hitPoint.m->getEmission();

	Vector3f L_dir(0.0f);
	Vector3f L_indir(0.0f);

	Intersection lightPoint;
	float pdf;
	sampleLight(lightPoint, pdf);

	Vector3f lightDir = lightPoint.coords - hitPoint.coords;
	float dis = dotProduct(lightDir, lightDir);
	Vector3f lightDirNormal = lightDir.normalized();
	Ray lightRay(hitPoint.coords, lightDirNormal);
	Intersection lightPointInter = intersect(lightRay);

	if (lightPointInter.happened && (lightPointInter.coords - lightPoint.coords).norm() < 0.01f)
	{
		Vector3f f_r = hitPoint.m->eval(ray.direction, lightDirNormal, hitPoint.normal);
		L_dir = lightPoint.emit * f_r * dotProduct(lightDirNormal, hitPoint.normal) * dotProduct(-lightDirNormal, lightPoint.normal) / dis / pdf;

	}
	if (get_random_float() > RussianRoulette)
		return L_dir;

	Vector3f reflectDir = hitPoint.m->sample(ray.direction, hitPoint.normal).normalized();
	Ray reflectRay(hitPoint.coords, reflectDir);
	Intersection reflectHitPoint = intersect(reflectRay);
	if (reflectHitPoint.happened && !reflectHitPoint.m->hasEmission())
	{
		float incidentPdf = hitPoint.m->pdf(ray.direction, reflectDir, hitPoint.normal);
		L_indir = castRay(reflectRay, depth + 1) * hitPoint.m->eval(ray.direction, reflectDir, hitPoint.normal) * dotProduct(reflectDir, hitPoint.normal) / incidentPdf / RussianRoulette;
	}
	//
	
	
	//		//L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
//		//	/ pdf(wo, wi, N) / RussianRoulette
	
	//if (!lightHitPoint.happened)
		//{
		//	Vector3f relectDir = reflect(ray.direction, hitPoint.normal);
		//	Vector3f outDir = hitPoint.m->sample(ray.direction, hitPoint.normal);
		//	//Vector3f f_r = hitPoint.m->eval(ray.direction, -1 * hitPointToLightDir, hitPoint.normal);
		//	float wsn = dotProduct(ray.direction_inv, -1 * hitPoint.normal);
		//	float wsnn = dotProduct(hitPointToLightDir, lightPoint.normal);
		//	float length = hitPointToLightDir.norm();
		//	Lo = lightPoint.emit * f_r * wsn * wsnn / (length * length) / pdf;
		//}
		//else
		//{
		//	float P_RR = get_random_float();
		//	if (P_RR < RussianRoulette)
		//	{

		//		Vector3f outDir = hitPoint.m->sample(-1 * ray.direction, hitPoint.normal);
		//		Ray outRay(ray(hitPoint.distance), outDir);
		//		float outPdf = hitPoint.m->pdf(outDir, -1 * ray.direction, hitPoint.normal);
		//		L_indir = castRay(outRay, depth - 1) * hitPoint.m->eval(outDir, -1 * ray.direction, hitPoint.normal) * dotProduct(-1 * ray.direction, hitPoint.normal) / outPdf / P_RR;
		//		//L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
		//		//	/ pdf(wo, wi, N) / RussianRoulette
		//	}
		//	else
		//	{
		//		return L_indir;
		//	}
		//}

	return L_dir + L_indir;
}