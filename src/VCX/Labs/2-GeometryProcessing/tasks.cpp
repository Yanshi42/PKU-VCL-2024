#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                int num = neighbors.size();
                float update;             
                if (num == 3){
                    update = 3.0f / 16.0f;
                }
                else{
                    update = 3.0f / (num * 8.0f);
                }
                glm::vec3 position = {0.0f, 0.0f, 0.0f};
                for(std::size_t j = 0; j < num; ++j){
                    position = position + prev_mesh.Positions[neighbors[j]];
                }
                position.r = position.r * update;
                position.g = position.g * update;
                position.b = position.b * update;
                position.r = position.r + (1.0f - num * update) * prev_mesh.Positions[i].r;
                position.g = position.g + (1.0f - num * update) * prev_mesh.Positions[i].g;
                position.b = position.b + (1.0f - num * update) * prev_mesh.Positions[i].b;
                curr_mesh.Positions.push_back(position);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 newPosition;
                    newPosition.r = (prev_mesh.Positions[e->To()].r + prev_mesh.Positions[e->From()].r) * 0.5f;
                    newPosition.g = (prev_mesh.Positions[e->To()].g + prev_mesh.Positions[e->From()].g) * 0.5f;
                    newPosition.b = (prev_mesh.Positions[e->To()].b + prev_mesh.Positions[e->From()].b) * 0.5f;
                    curr_mesh.Positions.push_back(newPosition);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 newPosition;
                    newPosition.r = (prev_mesh.Positions[e->To()].r + prev_mesh.Positions[e->From()].r) * 0.375f + 
                                    (prev_mesh.Positions[e->OppositeVertex()].r+prev_mesh.Positions[e->TwinOppositeVertex()].r) * 0.125f;
                    newPosition.g = (prev_mesh.Positions[e->To()].g + prev_mesh.Positions[e->From()].g) * 0.375f + 
                                    (prev_mesh.Positions[e->OppositeVertex()].g+prev_mesh.Positions[e->TwinOppositeVertex()].g) * 0.125f;
                    newPosition.b = (prev_mesh.Positions[e->To()].b + prev_mesh.Positions[e->From()].b) * 0.375f + 
                                    (prev_mesh.Positions[e->OppositeVertex()].b+prev_mesh.Positions[e->TwinOppositeVertex()].b) * 0.125f;
                    curr_mesh.Positions.push_back(newPosition);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    v0, m2, m1, v1, m0, m2, v2, m1, m0, m0, m1, m2
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        int maxi = input.Positions.size();
        glm::vec2 boundPoint;
        for(int i = 0; i < maxi; ++i) {
            if(G.Vertex(i)->OnBoundary()) {
                boundPoint = {input.Positions[i][0], input.Positions[i][1]};
                output.TexCoords[i] = glm::normalize(boundPoint) / 2.0f + 0.5f;
            }
        }

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            int num = 0;
            // glm::vec2 position = {0.0f, 0.0f};
            for(int i = 0; i < maxi; ++i) {
                auto vertex = G.Vertex(i);
                auto vertexNeighbors = vertex->Neighbors();
                int numNeighbors = vertexNeighbors.size();
                if(vertex->OnBoundary()) {
                    continue;
                }
                else {
                    glm::vec2 position = {0.0f, 0.0f};
                    for(int j = 0; j < numNeighbors; ++j) {
                        position += output.TexCoords[vertexNeighbors[j]];
                    }
                    output.TexCoords[i] = position / float(numNeighbors);
                }
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                // your code here:
                glm::mat3 position{output.Positions[f->VertexIndex(0)], output.Positions[f->VertexIndex(1)], output.Positions[f->VertexIndex(2)]};
                glm::vec3 tmp = {-1.0f, -1.0f, -1.0f};
                // position = glm::transpose({output.Positions[f->VertexIndex(0)], output.Positions[f->VertexIndex(1)], output.Positions[f->VertexIndex(2)]});
                // position = {output.Positions[f->VertexIndex(0)], output.Positions[f->VertexIndex(1)], output.Positions[f->VertexIndex(2)]};
                // position = glm::inverse(glm::transpose(position));
                position = glm::transpose(position);
                position = glm::inverse(position);
                tmp = position * tmp;
                glm::vec4 ker = {tmp, 1.0f};
                glm::vec4 zero = {0.0f, 0.0f, 0.0f, 0.0f};
                glm::mat4 zeroMat = {zero, zero, zero, zero};
                int threshold = 1000;
                double length = sqrt(glm::dot(tmp, tmp));
                if(length >= threshold) {
                    return zeroMat;
                }
                else {
                    ker = ker * (1.0f / float(length));
                    Kp = glm::outerProduct(ker, ker);
                    // Kp = {
                    //     ker[0]*ker[0], ker[0]*ker[1], ker[0]*ker[2], ker[0]*ker[3], 
                    //     ker[1]*ker[0], ker[1]*ker[1], ker[1]*ker[2], ker[1]*ker[3], 
                    //     ker[2]*ker[0], ker[2]*ker[1], ker[2]*ker[2], ker[2]*ker[3], 
                    //     ker[3]*ker[0], ker[3]*ker[1], ker[3]*ker[2], ker[3]*ker[3]
                    // };
                    return Kp;
                }
                // return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                glm::mat4 const & Qq = {
                    Q[0][0], Q[1][0], Q[2][0], 0.0f,
                    Q[0][1], Q[1][1], Q[2][1], 0.0f,
                    Q[0][2], Q[1][2], Q[2][2], 0.0f,
                    Q[0][3], Q[1][3], Q[2][3], 1.0f,
                };
                const float thresholdQq = 0.001f;
                ContractionPair out;
                out.edge = edge;
                if(glm::determinant(Qq) > thresholdQq) {
                    glm::vec4 kernel = {0.0f, 0.0f, 0.0f, 1.0f};
                    glm::vec4 targetPosition=glm::inverse(Qq) * kernel;
                    out.targetPosition=targetPosition;
                    out.cost=glm::dot(targetPosition, Q * targetPosition);
                }
                else {
                    // To be continued... 11.5
                    glm::vec4 beg = {p2, 1.0f};
                    glm::vec4 end = {p1, 1.0f};
                    glm::vec4 mid = (beg + end) / 2.0f;
                    double t1, t2, t3;
                    t1 = glm::dot(end, Q * end);
                    t2 = glm::dot(beg, Q * beg);
                    if(t1 >= t2) {
                        out.targetPosition = beg;
                        out.cost = t2;
                    }
                    else {
                        out.targetPosition = end;
                        out.cost = t1;
                    }
                    t3 = glm::dot(mid, Q * mid);
                    if(t3 <= out.cost) {
                        out.targetPosition = mid;
                        out.cost = t3;
                    }
                }
                return out;
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                auto newQ = UpdateQ(e->Face());
                auto preQ = Kf[G.IndexOf(e->Face())];
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                auto deltaQ = newQ - preQ;
                Qv[e->From()] += deltaQ;
                Qv[e->To()] += deltaQ;
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                Qv[v1] += newQ;
                //     4. Update $Kf$.
                Kf[G.IndexOf(e->Face())] = newQ;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for(auto edge : ring) {
                auto vertexBeg = edge->From();
                auto ringBeg = G.Vertex(vertexBeg)->Ring();
                for (auto edgeBeg : ringBeg) {
                    bool test = G.IsContractable(edgeBeg->NextEdge());
                    if(test) {
                        auto vertexEnd = edgeBeg->To();
                        auto pair11= MakePair(edgeBeg->NextEdge(), output.Positions[vertexEnd], output.Positions[vertexBeg], Qv[vertexEnd] + Qv[vertexBeg]);
                        pairs[pair_map[G.IndexOf(edgeBeg->NextEdge())]].targetPosition = pair11.targetPosition;
                        pairs[pair_map[G.IndexOf(edgeBeg->NextEdge())]].cost = pair11.cost; 
                    }
                    else {
                        pairs[pair_map[G.IndexOf(edgeBeg->NextEdge())]].edge = nullptr;
                    }
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                return 0.0f;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
    }
} // namespace VCX::Labs::GeometryProcessing
