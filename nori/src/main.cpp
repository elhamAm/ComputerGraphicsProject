/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/block.h>
#include <nori/gui.h>
#include <filesystem/path.h>
#include <nori/render.h>

#include <nori/render.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <filesystem/resolver.h>
#include <tbb/concurrent_vector.h>

#include <nori/common.h>
#include <thread>
#include <nori/block.h>
#include <atomic>

using namespace nori;
static void renderBlock2Scenes(const Scene* scene, const Scene* scene2, Sampler* sampler, ImageBlock& block) {
    const Camera* camera = scene->getCamera();
    const Integrator* integrator = scene->getIntegrator();

    Point2i offset = block.getOffset();
    Vector2i size = block.getSize();

    /* Clear the block contents */
    block.clear();

    /* For each pixel and pixel sample sample */
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Point2f pixelSample = Point2f((float)(x + offset.x()), (float)(y + offset.y())) + sampler->next2D();
            //Point2f apertureSample = sampler->next2D();

            Point2f pixelSample2 = Point2f((float)(x + offset.x()), (float)(y + offset.y())) + sampler->next2D();
            //Point2f apertureSample2 = sampler->next2D();

            /* Sample a ray from the camera */
            Ray3f ray;
            Ray3f ray2;
            Color3f value = camera->sampleRay(ray, pixelSample, sampler);
            Color3f value2 = camera->sampleRay(ray2, pixelSample2, sampler);

            /* Compute the incident radiance */
            value *= integrator->Li(scene, sampler, ray);
            value2 *= integrator->Li(scene2, sampler, ray2);

            /* Store in the image block */
            Color3f res = (value + value2) / 2;
            block.put(pixelSample, res);
        }
    }
}

static void renderBlock(const Scene* scene, Sampler* sampler, ImageBlock& block) {
    const Camera* camera = scene->getCamera();
    const Integrator* integrator = scene->getIntegrator();

    Point2i offset = block.getOffset();
    Vector2i size = block.getSize();

    /* Clear the block contents */
    block.clear();

    /* For each pixel and pixel sample sample */
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Point2f pixelSample = Point2f((float)(x + offset.x()), (float)(y + offset.y())) + sampler->next2D();
            //Point2f apertureSample = sampler->next2D();

            /* Sample a ray from the camera */
            Ray3f ray;
            Color3f value = camera->sampleRay(ray, pixelSample, sampler);

            /* Compute the incident radiance */
            value *= integrator->Li(scene, sampler, ray);

            /* Store in the image block */
            block.put(pixelSample, value);
        }
    }
}

void renderNoGuiAndInterpolate(const std::string& filename, const std::string& file2) {
    std::thread m_render_thread;
    std::atomic<int> m_render_status; // 0: free, 1: busy, 2: interruption, 3: done
    m_render_status = 0;

    filesystem::path path(filename);
    filesystem::path path2(file2);

    /* Add the parent directory of the scene file to the
       file resolver. That way, the XML file can reference
       resources (OBJ files, textures) using relative paths */
    getFileResolver()->prepend(path.parent_path());
    getFileResolver()->prepend(path2.parent_path());

    NoriObject* root = loadFromXML(filename);
    NoriObject* root2 = loadFromXML(file2);
    if (root->getClassType() == NoriObject::EScene) {
        Scene* m_scene = static_cast<Scene*>(root);
        Scene* m_scene2 = static_cast<Scene*>(root2);


        const nori::Camera* camera_ = m_scene->getCamera();


        m_scene->getIntegrator()->preprocess(m_scene);
        m_scene2->getIntegrator()->preprocess(m_scene2);

        NoriScreen* screen = nullptr;

        /* Allocate memory for the entire output image and clear it */
        nori::ImageBlock m_block(camera_->getOutputSize(), camera_->getReconstructionFilter());
        m_block.clear();

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);
        outputName += ".exr";

        /* Do the following in parallel and asynchronously */
        m_render_status = 1;
        m_render_thread = std::thread([&] {
            const Camera* camera = m_scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            //std::cout << "before sampler" << endl;
            auto numSamples = m_scene->getSampler()->getSampleCount();
            auto numBlocks = blockGenerator.getBlockCount();

            tbb::concurrent_vector< std::unique_ptr<Sampler> > samplers;
            samplers.resize(numBlocks);
            //std::cout << "before for" << endl;
            for (uint32_t k = 0; k < numSamples; ++k) {

                if (m_render_status == 2)
                    break;

                tbb::blocked_range<int> range(0, numBlocks);

                auto map = [&](const tbb::blocked_range<int>& range) {
                    // Allocate memory for a small image block to be rendered by the current thread
                    ImageBlock block(Vector2i(NORI_BLOCK_SIZE),
                        camera->getReconstructionFilter());

                    for (int i = range.begin(); i < range.end(); ++i) {
                        // Request an image block from the block generator
                        blockGenerator.next(block);

                        // Get block id to continue using the same sampler
                        auto blockId = block.getBlockId();
                        if (k == 0) { // Initialize the sampler for the first sample
                            std::unique_ptr<Sampler> sampler(m_scene->getSampler()->clone());
                            sampler->prepare(block);
                            samplers.at(blockId) = std::move(sampler);
                        }

                        // Render all contained pixels
                        renderBlock2Scenes(m_scene, m_scene2, samplers.at(blockId).get(), block);

                        // The image block has been processed. Now add it to the "big" block that represents the entire image
                        m_block.put(block);
                    }
                };

                /// Uncomment the following line for single threaded rendering
                //map(range);

                /// Default: parallel rendering
                tbb::parallel_for(range, map);

                blockGenerator.reset();
            }

            cout << "done. (took " << timer.elapsedString() << ")" << endl;

            /* Now turn the rendered image block into
                a properly normalized bitmap */
            m_block.lock();
            std::unique_ptr<Bitmap> bitmap(m_block.toBitmap());
            m_block.unlock();

            /* Save using the OpenEXR format */
            bitmap->save(outputName);

            delete m_scene;
            m_scene = nullptr;

            m_render_status = 3;
            });

        m_render_thread.join();

    }
    else {
        delete root;
    }

}

// no gui in this case
void renderNoGui(const std::string& filename) {
    std::thread m_render_thread;
    std::atomic<int> m_render_status; // 0: free, 1: busy, 2: interruption, 3: done
    m_render_status = 0;

    filesystem::path path(filename);

    /* Add the parent directory of the scene file to the
       file resolver. That way, the XML file can reference
       resources (OBJ files, textures) using relative paths */
    getFileResolver()->prepend(path.parent_path());

    NoriObject* root = loadFromXML(filename);
    if (root->getClassType() == NoriObject::EScene) {
        Scene* m_scene = static_cast<Scene*>(root);


        const nori::Camera* camera_ = m_scene->getCamera();
        m_scene->getIntegrator()->preprocess(m_scene);

        NoriScreen* screen = nullptr;

        /* Allocate memory for the entire output image and clear it */
        nori::ImageBlock m_block(camera_->getOutputSize(), camera_->getReconstructionFilter());
        m_block.clear();

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);
        outputName += ".exr";

        /* Do the following in parallel and asynchronously */
        m_render_status = 1;
        m_render_thread = std::thread([&] {
            const Camera* camera = m_scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            std::cout << "before sampler" << endl;
            auto numSamples = m_scene->getSampler()->getSampleCount();
            auto numBlocks = blockGenerator.getBlockCount();

            tbb::concurrent_vector< std::unique_ptr<Sampler> > samplers;
            samplers.resize(numBlocks);
            std::cout << "before for" << endl;
            for (uint32_t k = 0; k < numSamples; ++k) {

                if (m_render_status == 2)
                    break;

                tbb::blocked_range<int> range(0, numBlocks);

                auto map = [&](const tbb::blocked_range<int>& range) {
                    // Allocate memory for a small image block to be rendered by the current thread
                    ImageBlock block(Vector2i(NORI_BLOCK_SIZE),
                        camera->getReconstructionFilter());

                    for (int i = range.begin(); i < range.end(); ++i) {
                        // Request an image block from the block generator
                        blockGenerator.next(block);

                        // Get block id to continue using the same sampler
                        auto blockId = block.getBlockId();
                        if (k == 0) { // Initialize the sampler for the first sample
                            std::unique_ptr<Sampler> sampler(m_scene->getSampler()->clone());
                            sampler->prepare(block);
                            samplers.at(blockId) = std::move(sampler);
                        }

                        // Render all contained pixels
                        renderBlock(m_scene, samplers.at(blockId).get(), block);

                        // The image block has been processed. Now add it to the "big" block that represents the entire image
                        m_block.put(block);
                    }
                };

                /// Uncomment the following line for single threaded rendering
                //map(range);

                /// Default: parallel rendering
                tbb::parallel_for(range, map);

                blockGenerator.reset();
            }

            cout << "done. (took " << timer.elapsedString() << ")" << endl;

            /* Now turn the rendered image block into
                a properly normalized bitmap */
            m_block.lock();
            std::unique_ptr<Bitmap> bitmap(m_block.toBitmap());
            m_block.unlock();

            /* Save using the OpenEXR format */
            bitmap->save(outputName);

            delete m_scene;
            m_scene = nullptr;

            m_render_status = 3;
            });

        m_render_thread.join();

    }
    else {
        delete root;
    }

}

int main(int argc, char** argv) {
    using namespace nori;

    try {

        if (argc == 4) {
            //std::cout << "here 4" << endl;
            std::string filename = argv[1];
            std::string file2 = argv[2];
            filesystem::path path(filename);
            filesystem::path path2(file2);
            renderNoGuiAndInterpolate(filename, file2);
        }
        else if (argc == 3) {
            //std::cout << "here 3" << endl;
            std::string filename = argv[1];
            filesystem::path path(filename);
            renderNoGui(filename);
        }
        // if file is passed as argument, handle it
        else if (argc == 2) {
            //std::cout << "here 2" << endl;
            nanogui::init();
            ImageBlock block(Vector2i(720, 720), nullptr);
            NoriScreen* screen = new NoriScreen(block);
            std::string filename = argv[1];
            filesystem::path path(filename);

            if (path.extension() == "xml") {
                /* Render the XML scene file */

                screen->openXML(filename);

            }
            else if (path.extension() == "exr") {
                /* Alternatively, provide a basic OpenEXR image viewer */
                screen->openEXR(filename);
            }
            else {
                cerr << "Error: unknown file \"" << filename
                    << "\", expected an extension of type .xml or .exr" << endl;
            }

            nanogui::mainloop();
            delete screen;
            nanogui::shutdown();
        }



    }
    catch (const std::exception & e) {
        cerr << "Fatal error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
