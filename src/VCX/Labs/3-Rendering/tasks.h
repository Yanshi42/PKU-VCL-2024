#pragma once

#include <numeric>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/3-Rendering/intersect.h"
#include "Labs/3-Rendering/Ray.h"
#include "Labs/3-Rendering/BVH.h"

namespace VCX::Labs::Rendering {

    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    /* Optional: write your own accelerated intersector here */
struct BVHRayIntersector {
    Engine::Scene const * InternalScene = nullptr;
    BVH                   bvh;

    BVHRayIntersector() = default;

    // 初始化
    void InitScene(Engine::Scene const * scene) {
        InternalScene = scene;

        if (!InternalScene) {
            spdlog::warn("BVHRayIntersector::InitScene: No scene provided.");
            return;
        }

        // 预留内存
        size_t totalFaces = 0;
        for (const auto & model : InternalScene->Models) {
            totalFaces += model.Mesh.Indices.size() / 3;
        }

        std::vector<BVH::Face> faces;
        faces.reserve(totalFaces);

        // 收集mesh
        for (const auto & model : InternalScene->Models) {
            const auto & indices = model.Mesh.Indices;
            for (size_t i = 0; i < indices.size(); i += 3) {
                faces.emplace_back(&model, indices.data() + i);
            }
        }

        // 构建BVH
        bvh.Clear();
        bvh.Build(faces);
    }

    RayHit IntersectRay(Ray const & ray) const {
        RayHit result;

        if (!InternalScene) {
            spdlog::warn("BVHRayIntersector::IntersectRay: Uninitialized intersector.");
            result.IntersectState = false;
            return result;
        }

        Intersection its;
        BVH::Face hitFace;

        // 确定最近相交
        if (!bvh.FindIntersection(its, hitFace, ray)) {
            result.IntersectState = false;
            return result;
        }

        const auto & model = *hitFace.model;
        const auto & indices = hitFace.indice;
        const auto & positions = model.Mesh.Positions;
        const auto & normals = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
        const auto & texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();

        // 获取点面
        glm::vec3 p1 = positions[indices[0]];
        glm::vec3 p2 = positions[indices[1]];
        glm::vec3 p3 = positions[indices[2]];

        glm::vec3 n1 = normals[indices[0]];
        glm::vec3 n2 = normals[indices[1]];
        glm::vec3 n3 = normals[indices[2]];

        glm::vec2 uv1 = texcoords[indices[0]];
        glm::vec2 uv2 = texcoords[indices[1]];
        glm::vec2 uv3 = texcoords[indices[2]];

        // 计算纹理
        float w1 = 1.0f - its.u - its.v;
        glm::vec2 uvCoord = w1 * uv1 + its.u * uv2 + its.v * uv3;
        result.IntersectPosition = w1 * p1 + its.u * p2 + its.v * p3;
        result.IntersectNormal = glm::normalize(w1 * n1 + its.u * n2 + its.v * n3);
        const auto & material = InternalScene->Materials[model.MaterialIndex];
        result.IntersectMode = material.Blend;
        result.IntersectAlbedo = GetAlbedo(material, uvCoord);
        result.IntersectMetaSpec = GetTexture(material.MetaSpec, uvCoord);

        result.IntersectState = true;
        return result;
    }
};


    using RayIntersector = BVHRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

}
// namespace VCX::Labs::Rendering