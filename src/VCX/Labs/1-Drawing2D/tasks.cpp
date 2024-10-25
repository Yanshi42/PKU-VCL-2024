#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

#include <iostream>

using namespace std;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    float generateRandomDisturbance() {

        std::random_device rd;
        std::mt19937 gen(rd());
        
        std::uniform_real_distribution<float> dis(-0.5, 0.5);
        
        return dis(gen);
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {

        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y) + generateRandomDisturbance();
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {

        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 adjustment = noise.At(x, y) - 0.5f;
                glm::vec3 color = input.At(x, y) + adjustment;
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    int rankXY1(
        ImageRGB const & input,
        int              x,
        int              y){
        
        float color = input.At(x, y).r*0.299f +input.At(x, y).g*0.587f + input.At(x, y).b*0.114f;
        float colorXY = color * 10;
        if      (colorXY >= 0 && colorXY < 1)    return 0;
        else if (colorXY >= 1 && colorXY < 2)    return 1;
        else if (colorXY >= 2 && colorXY < 3)    return 2;
        else if (colorXY >= 3 && colorXY < 4)    return 3;
        else if (colorXY >= 4 && colorXY < 5)    return 4;
        else if (colorXY >= 5 && colorXY < 6)    return 5;
        else if (colorXY >= 6 && colorXY < 7)    return 6;
        else if (colorXY >= 7 && colorXY < 8)    return 7;
        else if (colorXY >= 8 && colorXY < 9)    return 8;
        else if (colorXY >= 9)                   return 9;
        return -1;
    }

        int rankXY2(
        ImageRGB const & input,
        int              x,
        int              y){
        
        float color = input.At(x, y).r*0.299f +input.At(x, y).g*0.587f + input.At(x, y).b*0.114f;
        float colorXY = color * 10;
        if      (colorXY > 0 && colorXY <= 1)    return 0;
        else if (colorXY > 1 && colorXY <= 2)    return 1;
        else if (colorXY > 2 && colorXY <= 3)    return 2;
        else if (colorXY > 3 && colorXY <= 4)    return 3;
        else if (colorXY > 4 && colorXY <= 5)    return 4;
        else if (colorXY > 5 && colorXY <= 6)    return 5;
        else if (colorXY > 6 && colorXY <= 7)    return 6;
        else if (colorXY > 7 && colorXY <= 8)    return 7;
        else if (colorXY > 8 && colorXY <= 9)    return 8;
        else if (colorXY > 9)                   return 9;
        return -1;
    }

    const int ditherMatrix[10][3][3] = {
    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
    {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}},
    {{0, 0, 0}, {0, 0, 0}, {1, 1, 0}},
    {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}},
    {{0, 1, 0}, {0, 1, 0}, {1, 1, 0}},
    {{0, 1, 0}, {1, 1, 0}, {1, 1, 0}},
    {{0, 1, 0}, {1, 1, 1}, {1, 1, 0}},
    {{1, 1, 0}, {1, 1, 1}, {1, 1, 0}},
    {{1, 1, 1}, {1, 1, 1}, {1, 1, 0}},
    {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    };

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                int rank = rankXY1(input, x, y);
                for (int dx = 0; dx < 3; ++dx) {
                    for (int dy = 0; dy < 3; ++dy) {
                        glm::vec3 color = (ditherMatrix[rank][dx][dy] == 1)
                                            ? glm::vec3(1.0f, 1.0f, 1.0f)
                                            : glm::vec3(0.0f, 0.0f, 0.0f);
                        output.At(x*3+dx, y*3+dy) = color;
                    }
            }
        }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) 
    {
        std::size_t width = input.GetSizeX();
        std::size_t height = input.GetSizeY();
        ImageRGB errorImage = input;

        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                glm::vec3 color = errorImage.At(x, y);
                glm::vec3 newColor = {
                    color.r > 0.5f ? 1.0f : 0.0f,
                    color.g > 0.5f ? 1.0f : 0.0f,
                    color.b > 0.5f ? 1.0f : 0.0f
                };
                output.At(x, y) = newColor;
                glm::vec3 error = color - newColor;

                if (x + 1 < width) {
                    glm::vec3 rightPixel = errorImage.At(x + 1, y);
                    errorImage.At(x + 1, y) = rightPixel + error * (7.0f / 16.0f);
                }
                if (x > 0 && y + 1 < height) {
                    glm::vec3 leftBottomPixel = errorImage.At(x - 1, y + 1);
                    errorImage.At(x - 1, y + 1) = leftBottomPixel + error * (3.0f / 16.0f);
                }
                if (y + 1 < height) {
                    glm::vec3 bottomPixel = errorImage.At(x, y + 1);
                    errorImage.At(x, y + 1) = bottomPixel + error * (5.0f / 16.0f);
                }
                if (x + 1 < width && y + 1 < height) {
                    glm::vec3 rightBottomPixel = errorImage.At(x + 1, y + 1);
                    errorImage.At(x + 1, y + 1) = rightBottomPixel + error * (1.0f / 16.0f);
                }
            }
        }
    }

    /******************* 2.Image Filtering *****************/

    const float kernel[3][3] = {
        {1 / 9.0f, 1 / 9.0f, 1 / 9.0f},
        {1 / 9.0f, 1 / 9.0f, 1 / 9.0f},
        {1 / 9.0f, 1 / 9.0f, 1 / 9.0f}
    };

    const int kernelX[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1}
    };

    const int kernelY[3][3] = {
        {-1, -2, -1},
        { 0,  0,  0},
        { 1,  2,  1}
    };

    const int kernelC[3][3] = {
        {1,  1,  1},
        {1, -8,  1},
        {1,  1,  1}
    };

    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        
        int width = input.GetSizeX();
        int height = input.GetSizeY();

        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                float r = 0, g = 0, b = 0;
                for (int ky = -1; ky <= 1; ++ky) {
                    for (int kx = -1; kx <= 1; ++kx) {
                        auto pixel = input.At(x + kx, y + ky);
                        r += pixel.r * kernel[ky + 1][kx + 1];
                        g += pixel.g * kernel[ky + 1][kx + 1];
                        b += pixel.b * kernel[ky + 1][kx + 1];
                    }
                }
                output.At(x, y) = {r, g, b};
            }
        }
    }

// void Edge(
//     ImageRGB &       output,
//     ImageRGB const & input) {

//     int width = input.GetSizeX();
//     int height = input.GetSizeY();
//     for (int y = 1; y < height - 1; ++y) {
//         for (int x = 1; x < width - 1; ++x) {
//             float rX = 0, gX = 0, bX = 0;
//             float rY = 0, gY = 0, bY = 0;

//             for (int ky = -1; ky <= 1; ++ky) {
//                 for (int kx = -1; kx <= 1; ++kx) {
//                     auto pixel = input.At(x + kx, y + ky);
//                     rX += pixel.r * kernelX[ky + 1][kx + 1];
//                     gX += pixel.g * kernelX[ky + 1][kx + 1];
//                     bX += pixel.b * kernelX[ky + 1][kx + 1];

//                     rY += pixel.r * kernelY[ky + 1][kx + 1];
//                     gY += pixel.g * kernelY[ky + 1][kx + 1];
//                     bY += pixel.b * kernelY[ky + 1][kx + 1];
//                 }
//             }

//             int r = static_cast<int>(std::sqrt(rX * rX + rY * rY));
//             int g = static_cast<int>(std::sqrt(gX * gX + gY * gY));
//             int b = static_cast<int>(std::sqrt(bX * bX + bY * bY));

//             r = std::min(255, std::max(0, r));
//             g = std::min(255, std::max(0, g));
//             b = std::min(255, std::max(0, b));

//             output.At(x, y) = {r, g, b};
//         }
//     }
// }

void Edge(
    ImageRGB &       output,
    ImageRGB const & input) {

    int width = input.GetSizeX();
    int height = input.GetSizeY();
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            float r = 0, g = 0, b = 0;

            for (int ky = -1; ky <= 1; ++ky) {
                for (int kx = -1; kx <= 1; ++kx) {
                    auto pixel = input.At(x + kx, y + ky);
                    r += pixel.r * kernelC[ky + 1][kx + 1];
                    g += pixel.g * kernelC[ky + 1][kx + 1];
                    b += pixel.b * kernelC[ky + 1][kx + 1];
                }
            }

            output.At(x, y) = {r, g, b};
        }
    }
}

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);

        // for (std::size_t y = 0; y < height; ++y) {
        //     g[y * width] = inputBack.At(offset.x, offset.y + y);
        //     g[y * width + width - 1] = inputBack.At(offset.x + width - 1, offset.y + y);
        // }
        // for (std::size_t x = 0; x < width; ++x) {
        //     g[x] = inputBack.At(offset.x + x, offset.y);
        //     g[(height - 1) * width + x] = inputBack.At(offset.x + x, offset.y + height - 1);
        // }

        // for (int iter = 0; iter < 8000; ++iter) {
        //     for (std::size_t y = 1; y < height - 1; ++y) {
        //         for (std::size_t x = 1; x < width - 1; ++x) {
        //             glm::vec3 laplace =
        //                 g[(y - 1) * width + x] + g[(y + 1) * width + x] +
        //                 g[y * width + x - 1] + g[y * width + x + 1] - 4.0f * g[y * width + x];
                    
        //             glm::vec3 frontGrad = inputFront.At(x, y) - inputFront.At(x - 1, y) + inputFront.At(x + 1, y) - inputFront.At(x, y - 1) + inputFront.At(x, y + 1);

        //             g[y * width + x] = (laplace + frontGrad) * 0.25f;
        //         }
        //     }
        // }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        int x0 = p0[0], y0 = p0[1], x1 = p1[0], y1 = p1[1];
        if (abs(x1 - x0) >= abs(y1 - y0)){
            if (x1 == x0){
                if (y0 < y1)    {for (int y = y0; y <= y1; y++) canvas.At(x0, y) = color;}
                else            {for (int y = y1; y <= y0; y++) canvas.At(x0, y) = color;}
            }
            else {
                int dx = 2 * abs(x1 - x0);
                int dy = 2 * abs(y1 - y0);
                int sx = (x1 >= x0) ? 1 : -1;
                int sy = (y1 >= y0) ? 1 : -1;
                int F = dy - dx / 2;
                int y = y0;

                for (int x = x0; x != x1; x += sx){
                    canvas.At(x, y) = color;
                    if (F < 0) {
                        F += dy;
                    } else {
                        y += sy;
                        F += dy - dx;
                    }
                }
                canvas.At(x1, y1) = color;
            }
        }
        else{
            if(y1 == y0){
                if (x0 < x1)    {for (int x = x0; x <= x1; x++) canvas.At(x, y0) = color;}
                else            {for (int x = x1; x <= x0; x++) canvas.At(x, y0) = color;}
            }
            else {
                int dx = 2 * abs(x1 - x0);
                int dy = 2 * abs(y1 - y0);
                int sx = (x1 >= x0) ? 1 : -1;
                int sy = (y1 >= y0) ? 1 : -1;
                int F = dx - dy / 2;
                int x = x0;

                for (int y = y0; y != y1; y += sy){
                    canvas.At(x, y) = color;
                    if (F < 0) {
                        F += dx;
                    } else {
                        x += sx;
                        F += dx - dy;
                    }
                }
                canvas.At(x1, y1) = color;
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
    bool IsInsideTriangle(
        glm::ivec2 const A,
        glm::ivec2 const B,
        glm::ivec2 const C,
        int x, int y) {
        double resultAB = (B.x - A.x) * (y - A.y) - (B.y - A.y) * (x - A.x);
        double resultBC = (C.x - B.x) * (y - B.y) - (C.y - B.y) * (x - B.x);
        double resultCA = (A.x - C.x) * (y - C.y) - (A.y - C.y) * (x - C.x);
        if (resultAB >= 0 && resultBC >= 0 && resultCA >= 0 || resultAB < 0 && resultBC < 0 && resultCA < 0)
            return TRUE;
        else
            return FALSE;
    }
    
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        for (std::size_t x = 0; x < canvas.GetSizeX(); ++x)
            for (std::size_t y = 0; y < canvas.GetSizeY(); ++y) {
                if (IsInsideTriangle(p0, p1, p2, x, y))
                    canvas.At(x, y) = color;
                else
                    continue;
            };
        }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        
        int ow = output.GetSizeX();
        int oh = output.GetSizeY();
        int iw = input.GetSizeX();
        int ih = input.GetSizeY();

        for (int y = 0; y < oh; ++y) {
            for (int x = 0; x < ow; ++x) {
                glm::vec3 colorSum(0.0f, 0.0f, 0.0f);

                for (int dy = 0; dy < rate; ++dy) {
                    for (int dx = 0; dx < rate; ++dx) {
                        float sampleX = (x*rate+dx)*(iw/float(ow*rate));
                        float sampleY = (y*rate+dy)*(ih/float(oh*rate));
                        glm::ivec2 samplePosition = glm::ivec2(int(sampleX), int(sampleY));
                        glm::vec3 sampleColor = input.At(samplePosition.x, samplePosition.y);
                        colorSum += sampleColor;
                    }
                }

                glm::vec3 averageColor = colorSum/float(rate*rate);
                output.At(x, y) = averageColor;
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        
        size_t n = points.size();
        if (n == 1) return points[0];
        std::vector<glm::vec2> newPoints(n-1);
        for (size_t i = 0; i < n-1; ++i) newPoints[i] = (1-t)*points[i]+t*points[i+1];
        return CalculateBezierPoint(std::span<glm::vec2>(newPoints), t);
    }
} // namespace VCX::Labs::Drawing2D