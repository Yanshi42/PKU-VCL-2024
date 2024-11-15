#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <limits>

namespace VCX::Labs::Rendering {

    class BVH {
    public:
        struct Face {
            VCX::Engine::Model const * model;
            std::uint32_t const *      indice;

            Face() = default;
            Face(VCX::Engine::Model const * _model, std::uint32_t const * _indice)
                : model(_model), indice(_indice) {}

            std::pair<float, float> range(int dim) const {
                float mn = std::min({model->Mesh.Positions[indice[0]][dim], 
                                     model->Mesh.Positions[indice[1]][dim], 
                                     model->Mesh.Positions[indice[2]][dim]});
                float mx = std::max({model->Mesh.Positions[indice[0]][dim], 
                                     model->Mesh.Positions[indice[1]][dim], 
                                     model->Mesh.Positions[indice[2]][dim]});
                return {mn, mx};
            }

            std::pair<glm::vec3, glm::vec3> range() const {
                glm::vec3 mn = model->Mesh.Positions[indice[0]];
                glm::vec3 mx = mn;
                for (int i = 1; i < 3; i++) {
                    const glm::vec3 &p = model->Mesh.Positions[indice[i]];
                    mn = glm::min(mn, p);
                    mx = glm::max(mx, p);
                }
                return {mn, mx};
            }

            const glm::vec3 & point(int idx) const {
                return model->Mesh.Positions[indice[idx]];
            }
        };

        struct Node {
            glm::vec3 min_pos;
            glm::vec3 max_pos;
            bool      is_leaf = false; // 确保初始值
            union {
                Node *son[2] = {nullptr, nullptr}; // 确保初始化
                Face  face;
            };
        };

    private:
        std::vector<Face> internelFaces;
        Node *            root = nullptr;

        // 构建树函数
        void BuildTree(Node *&node, int L, int R) {
            node = new Node();
            auto [mn, mx] = internelFaces[L].range();
            node->min_pos = mn;
            node->max_pos = mx;

            if (L + 1 == R) { // 单个面片，设为叶节点
                node->is_leaf = true;
                node->face = internelFaces[L];
                return;
            }
            node->is_leaf = false;

            for (int i = L + 1; i < R; i++) {
                auto [face_min, face_max] = internelFaces[i].range();
                node->min_pos = glm::min(node->min_pos, face_min);
                node->max_pos = glm::max(node->max_pos, face_max);
            }

            int split_dim = 0;
            glm::vec3 extents = node->max_pos - node->min_pos;
            if (extents.y > extents.x) split_dim = 1;
            if (extents.z > extents[split_dim]) split_dim = 2;

            int mid = (L + R) / 2;
            std::nth_element(internelFaces.begin() + L, internelFaces.begin() + mid, internelFaces.begin() + R,
                             [split_dim](const Face &a, const Face &b) {
                                 return a.range(split_dim).first < b.range(split_dim).first;
                             });

            BuildTree(node->son[0], L, mid);
            BuildTree(node->son[1], mid, R);
        }

        void free(Node *node) {
            if (!node) return;
            if (!node->is_leaf) {
                if (node->son[0]) free(node->son[0]);
                if (node->son[1]) free(node->son[1]);
            }
            delete node;
        }

        bool RayInBox(const Ray &ray, const glm::vec3 &min_pos, const glm::vec3 &max_pos) const {
            const float EPS = 1e-6f;
            float tmin = 0.0f, tmax = std::numeric_limits<float>::max();
            for (int i = 0; i < 3; i++) {
                if (fabs(ray.Direction[i]) < EPS) {
                    if (ray.Origin[i] < min_pos[i] || ray.Origin[i] > max_pos[i]) return false;
                } else {
                    float t1 = (min_pos[i] - ray.Origin[i]) / ray.Direction[i];
                    float t2 = (max_pos[i] - ray.Origin[i]) / ray.Direction[i];
                    if (t1 > t2) std::swap(t1, t2);
                    tmin = std::max(tmin, t1);
                    tmax = std::min(tmax, t2);
                    if (tmin > tmax) return false;
                }
            }
            return true;
        }

        bool FindInNode(Node *node, Intersection &result, Face &resFace, const Ray &ray) const {
            if (!node) return false;

            if (node->is_leaf) {
                if (IntersectTriangle(result, ray, node->face.point(0), node->face.point(1), node->face.point(2))) {
                    resFace = node->face;
                    return true;
                }
                return false;
            }

            if (!RayInBox(ray, node->min_pos, node->max_pos)) return false;

            Intersection res1, res2;
            Face resf1, resf2;
            bool hit1 = FindInNode(node->son[0], res1, resf1, ray);
            bool hit2 = FindInNode(node->son[1], res2, resf2, ray);

            if (hit1 && hit2) {
                if (res1.t < res2.t) result = res1, resFace = resf1;
                else result = res2, resFace = resf2;
            } else if (hit1) result = res1, resFace = resf1;
            else if (hit2) result = res2, resFace = resf2;
            else return false;

            return true;
        }

    public:
        BVH() = default;
        ~BVH() { Clear(); }

        void Clear() {
            if (root) {
                free(root);
                root = nullptr;
            }
            internelFaces.clear();
        }

        void Build(std::vector<Face> faces) {
            Clear();
            internelFaces = std::move(faces);
            BuildTree(root, 0, internelFaces.size());
        }

        bool FindIntersection(Intersection &result, Face &resFace, const Ray &ray) const {
            return FindInNode(root, result, resFace, ray);
        }
    };
}
