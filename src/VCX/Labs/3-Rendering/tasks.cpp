#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        // glm::vec3 edge1, edge2, tvec, pvec, qvec;
        // double det, inv_det;
        // edge1 = p2 - p1;
        // edge2 = p3 - p1;
        // pvec = glm::cross(edge1, edge2);
        // det = glm::dot(edge1, edge2);
        float epsilon1 = 1e-6, epsilon2 = 1e-2;
        glm::vec3 normal = glm::cross(p1 - p2, p1 - p3);
        // float dot = glm::dot(p2 - p1, p3 - p1);
        if(abs(glm::dot(ray.Direction, normal)) < epsilon1) {
            return false;
        }
        output.t = glm::dot(p1 - ray.Origin, normal) / glm::dot(ray.Direction, normal);
        if(output.t <= epsilon2) {
            return false;
        }
        output.u = glm::dot(p3 - p1, glm::cross(ray.Direction, ray.Origin - p1)) / glm::dot(ray.Direction, normal);
        output.v = glm::dot(p1 - p2, glm::cross(ray.Direction, ray.Origin - p1)) / glm::dot(ray.Direction, normal);
        if (output.u < 0 || output.v > 1 || output.v < 0 || output.v > 1 || output.u + output.v > 1)
            return false;
        return true;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            glm::vec3 normal = glm::normalize(n);
            glm::vec3 view = -ray.Direction;
            glm::vec3 L;
            result += kd * 0.06f;

            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    L           = glm::normalize(l);
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        // 渲染速度过慢，做一些优化
                        // glm::vec3 newPosition = pos;
                        // while(1) {
                        //     auto newIntersection = intersector.IntersectRay(Ray(newPosition, glm::normalize(l)));
                        //     if(!newIntersection.IntersectState) {
                        //         break;
                        //     }
                        //     else {
                        //         glm::vec3 newIntersectionPos = newIntersection.IntersectPosition;
                        //         if(glm::dot(l, l) <= glm::dot(newIntersectionPos - pos, l)) {
                        //             break;
                        //         }
                        //         float w = newIntersection.IntersectAlbedo.w;
                        //         if(w >= 0.2) {
                        //             attenuation = attenuation * (1 - w);
                        //             break;
                        //         }
                        //         else {
                        //             newPosition = newIntersectionPos;
                        //         }
                        //     }
                        // }
                        auto newIntersection = intersector.IntersectRay(Ray(pos, L));
                        // glm::vec3 newIntersectionPos = newIntersection.IntersectPosition;
                        while(newIntersection.IntersectState && newIntersection.IntersectAlbedo.w < 0.2) {
                            newIntersection = intersector.IntersectRay(Ray(newIntersection.IntersectPosition, L));
                        }
                        if(newIntersection.IntersectState) {
                            glm::vec3 shadow = newIntersection.IntersectPosition - pos;
                            if(glm::dot(shadow, shadow) < glm::dot(l, l)) {
                                attenuation = 0.0f;
                            }
                        }
                    }
                }
                else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    L           = glm::normalize(l);
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        // 渲染速度过慢，做一些优化
                        // glm::vec3 newPosition = pos;
                        // while(1) {
                        //     auto newIntersection = intersector.IntersectRay(Ray(newPosition, l));
                        //     if(!newIntersection.IntersectState) {
                        //         break;
                        //     }
                        //     else {
                        //         glm::vec3 newIntersectionPos = newIntersection.IntersectPosition;
                        //         float w = newIntersection.IntersectAlbedo.w;
                        //         if(w >= 0.2) {
                        //             attenuation = attenuation * (1 - w);
                        //             break;
                        //         }
                        //         else {
                        //             newPosition = newIntersectionPos;
                        //         }
                        //     }
                        // }
                        auto newIntersection = intersector.IntersectRay(Ray(pos, L));
                        // glm::vec3 newIntersectionPos = newIntersection.IntersectPosition;
                        while(newIntersection.IntersectState && newIntersection.IntersectAlbedo.w < 0.2) {
                            newIntersection = intersector.IntersectRay(Ray(newIntersection.IntersectPosition, L));
                        }
                        if(newIntersection.IntersectState) {
                            // glm::vec3 shadow = newIntersection.IntersectPosition - pos;
                            // if(glm::dot(shadow, shadow) < glm::dot(l, l)) {
                            //     attenuation = 0.0f;
                            // }
                            attenuation = 0.0f;
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                glm::vec3 tmp = glm::normalize(view + L);
                float coD = glm::max(0.0f, glm::dot(L, n));
                float coS = glm::pow(glm::max(0.0f, dot(tmp, n)), shininess);
                result += (kd * coD + ks * coS) * attenuation * light.Intensity;
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering