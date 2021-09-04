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

#include <nori/render.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/denoising.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <filesystem/resolver.h>
#include <tbb/concurrent_vector.h>


NORI_NAMESPACE_BEGIN

RenderThread::RenderThread(ImageBlock& block) :
    m_block(block)
{
    m_render_status = 0;
    m_progress = 1.f;
}
RenderThread::~RenderThread() {
    stopRendering();
}

bool RenderThread::isBusy() {
    if (m_render_status == 3) {
        m_render_thread.join();
        m_render_status = 0;
    }
    return m_render_status != 0;
}

void RenderThread::stopRendering() {
    if (isBusy()) {
        cout << "Requesting interruption of the current rendering" << endl;
        m_render_status = 2;
        m_render_thread.join();
        m_render_status = 0;
        cout << "Rendering successfully aborted" << endl;
    }
}

float RenderThread::getProgress() {
    if (isBusy()) {
        return m_progress;
    }
    else return 1.f;
}

static void renderBlock(const Scene *scene, Sampler *sampler, ImageBlock &block, const float sampleTime) {
    const Camera *camera = scene->getCamera();
    const Integrator *integrator = scene->getIntegrator();

    Point2i offset = block.getOffset();
    Vector2i size = block.getSize();

    /* Clear the block contents */
    block.clear();
    

    /* For each pixel and pixel sample sample */
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Point2f pixelSample = Point2f((float)(x + offset.x()), (float)(y + offset.y())) + sampler->next2D();
            Point2f apertureSample = sampler->next2D();

            Color3f value = Color3f(0.0f);
            if (!camera->use_ca)
            {
                /* Sample a ray from the camera */
                Ray3f ray;
                value = camera->sampleRay(ray, pixelSample, sampler, sampleTime);

                /* Compute the incident radiance */
                value *= integrator->Li(scene, sampler, ray);


            }
            else
            {
                /* Sample a ray from the camera */
                Ray3f ray;
                
                value = Color3f(0.0f);
                // red
                Color3f value_r = camera->sampleRay(ray, pixelSample, sampler, sampleTime, 0);
                value += value_r * integrator->Li(scene, sampler, ray);
                // green
                Color3f value_g = camera->sampleRay(ray, pixelSample, sampler, sampleTime, 1);
                value += value_g * integrator->Li(scene, sampler, ray);
                // blue
                Color3f value_b = camera->sampleRay(ray, pixelSample, sampler, sampleTime, 2);
                value += value_b * integrator->Li(scene, sampler, ray);
            }

            /* Store in the image block */
            block.put(pixelSample, value);

        }
    }
}


static void renderBlock(const Scene* scene, Sampler* sampler, ImageBlock& block, ImageBlock& normal_block, ImageBlock& texture_block, ImageBlock& depth_block, ImageBlock& visibility_block) {
    const Camera* camera = scene->getCamera();
    const Integrator* integrator = scene->getIntegrator();

    Point2i offset = block.getOffset();
    Vector2i size = block.getSize();

    /* Clear the block contents */
    block.clear();
    normal_block.clear();
    texture_block.clear();
    depth_block.clear();
    visibility_block.clear();

    /* For each pixel and pixel sample sample */
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Point2f pixelSample = Point2f((float)(x + offset.x()), (float)(y + offset.y())) + sampler->next2D();

            /* Sample a ray from the camera */
            Ray3f ray;
            Color3f value = camera->sampleRay(ray, pixelSample, sampler);

            /* Compute the incident radiance */
            value *= integrator->Li(scene, sampler, ray);
            FeatureSet feature_set = FeatureGatherer::gather(scene, sampler, ray);

            /* Store in the image block */
            block.put(pixelSample, value);
            normal_block.put(pixelSample, Color3f(feature_set.normal.x(), feature_set.normal.y(), feature_set.normal.z()));
            texture_block.put(pixelSample, feature_set.texture);
            depth_block.put(pixelSample, Color3f(feature_set.depth));
            visibility_block.put(pixelSample, Color3f(feature_set.visibility));
        }
    }
}

static void ComputeVariacneAndUpdate(ImageBlock& m_block, ImageBlock& m_block_single_sample, ImageBlock& m_block_variance)
{
    Point2i offset = m_block.getOffset();
    Vector2i size = m_block.getSize();

    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Color3f delta = m_block_single_sample.coeffRef(y, x).divideByFilterWeight() - m_block.coeffRef(y, x).divideByFilterWeight();
            m_block.coeffRef(y, x) += m_block_single_sample.coeffRef(y, x);
            Color3f delta2 = m_block_single_sample.coeffRef(y, x).divideByFilterWeight() - m_block.coeffRef(y, x).divideByFilterWeight();
            Color3f production = delta * delta2;
            m_block_variance.coeffRef(y, x) += Color4f(production);
        }
    }
}

static void ComputeVariacneAndUpdate2(FeatureVarianceBlock& feature_variance_block, ImageBlock& feature_single)
{
    Point2i offset = feature_single.getOffset();
    Vector2i size = feature_single.getSize();
    ImageBlock& block1 = feature_variance_block.block1;
    ImageBlock& block2 = feature_variance_block.block2;



    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Color3f delta = feature_single.coeffRef(y, x).divideByFilterWeight() -
                Color4f(block1.coeffRef(y, x) + block2.coeffRef(y, x)).divideByFilterWeight();
            if (feature_variance_block.count % 2 == 0)
                block1.coeffRef(y, x) += feature_single.coeffRef(y, x);
            else
                block2.coeffRef(y, x) += feature_single.coeffRef(y, x);
            Color3f delta2 = feature_single.coeffRef(y, x).divideByFilterWeight() -
                Color4f(block1.coeffRef(y, x) + block2.coeffRef(y, x)).divideByFilterWeight();
            Color3f production = delta * delta2;
            feature_variance_block.variance_block.coeffRef(y, x) += Color4f(production);
        }
    }

    feature_variance_block.count++;
}

static void AverageVariance(ImageBlock& m_block_variance, int n)
{
    Vector2i size = m_block_variance.getSize();
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            m_block_variance.coeffRef(y, x) /= float(n - 1) * float(n);
            m_block_variance.coeffRef(y, x).w() = 1.0f;
        }
    }
}

static void SaveFeatureAndVariance(ImageBlock& block1, ImageBlock& block2, ImageBlock& block_variance, int numSamples, std::string feature_name, bool is_feature)
{
    block1.lock();
    std::unique_ptr<Bitmap> bitmap1(block1.toBitmap());
    block1.unlock();
    if (is_feature)
        bitmap1->save(feature_name + "1.exr");
    else
        bitmap1->save(feature_name + ".exr");


    if (is_feature)
    {
        block2.lock();
        std::unique_ptr<Bitmap> bitmap2(block2.toBitmap());
        block2.unlock();
        bitmap2->save(feature_name + "2.exr");
    }

    AverageVariance(block_variance, numSamples);
    block_variance.lock();
    std::unique_ptr<Bitmap> bitmap_variance(block_variance.toBitmap());
    block_variance.unlock();
    bitmap_variance->save(feature_name + "_variance.exr");
}




void RenderThread::renderScene(const std::string& filename) {

    filesystem::path path(filename);

    /* Add the parent directory of the scene file to the
       file resolver. That way, the XML file can reference
       resources (OBJ files, textures) using relative paths */
    //std::cout << "get file" << endl;
    getFileResolver()->prepend(path.parent_path());

    //std::cout << "root" << endl;
    NoriObject* root = loadFromXML(filename);

    // When the XML root object is a scene, start rendering it ..
    if (root->getClassType() == NoriObject::EScene) {
        m_scene = static_cast<Scene*>(root);

        const Camera* camera_ = m_scene->getCamera();
        m_scene->getIntegrator()->preprocess(m_scene);

        /* Allocate memory for the entire output image and clear it */
        m_block.init(camera_->getOutputSize(), camera_->getReconstructionFilter());
        m_block.clear();
        //std::cout << "after clear" << endl; 

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);
        outputName += ".exr";

        /* Do the following in parallel and asynchronously */
        m_render_status = 1;
        m_render_thread = std::thread([this, outputName] {
            const Camera* camera = m_scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);
            //std::cout << "after block generator" << endl;

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            auto numSamples = m_scene->getSampler()->getSampleCount();
            auto numBlocks = blockGenerator.getBlockCount();

            tbb::concurrent_vector< std::unique_ptr<Sampler> > samplers;
            samplers.resize(numBlocks);

            for (uint32_t k = 0; k < numSamples; ++k) {
                m_progress = k / float(numSamples);
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
                        renderBlock(m_scene, samplers.at(blockId).get(), block, ((float)k) / ((float)numSamples-1));

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

    }
    else {
        delete root;
    }

}




void RenderThread::renderSceneOutput(const std::string& filename) {

    filesystem::path path(filename);

    /* Add the parent directory of the scene file to the
       file resolver. That way, the XML file can reference
       resources (OBJ files, textures) using relative paths */
    getFileResolver()->prepend(path.parent_path());

    NoriObject* root = loadFromXML(filename);

    // When the XML root object is a scene, start rendering it ..
    if (root->getClassType() == NoriObject::EScene) {
        m_scene = static_cast<Scene*>(root);

        const Camera* camera_ = m_scene->getCamera();
        m_scene->getIntegrator()->preprocess(m_scene);

        /* Allocate memory for the entire output image and clear it */
        m_block.init(camera_->getOutputSize(), camera_->getReconstructionFilter());
        m_block.clear();

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);
        outputName += ".exr";

        /* Do the following in parallel and asynchronously */
        m_render_status = 1;
        m_render_thread = std::thread([this, outputName] {
            const Camera* camera = m_scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            auto numSamples = m_scene->getSampler()->getSampleCount();
            auto numBlocks = blockGenerator.getBlockCount();

            tbb::concurrent_vector< std::unique_ptr<Sampler> > samplers;
            samplers.resize(numBlocks);

            ImageBlock m_block_variance(camera->getOutputSize(), camera->getReconstructionFilter());
            FeatureVarianceBlock normal_FVB(camera->getOutputSize(), camera->getReconstructionFilter());
            FeatureVarianceBlock texture_FVB(camera->getOutputSize(), camera->getReconstructionFilter());
            FeatureVarianceBlock depth_FVB(camera->getOutputSize(), camera->getReconstructionFilter());
            FeatureVarianceBlock visibility_FVB(camera->getOutputSize(), camera->getReconstructionFilter());

            for (uint32_t k = 0; k < numSamples; ++k) {
                m_progress = k / float(numSamples);
                if (m_render_status == 2)
                    break;

                ImageBlock m_block_single_sample(camera->getOutputSize(), camera->getReconstructionFilter());
                ImageBlock normal_block_single(camera->getOutputSize(), camera->getReconstructionFilter());
                ImageBlock texture_block_single(camera->getOutputSize(), camera->getReconstructionFilter());
                ImageBlock depth_block_single(camera->getOutputSize(), camera->getReconstructionFilter());
                ImageBlock visibility_block_single(camera->getOutputSize(), camera->getReconstructionFilter());

                tbb::blocked_range<int> range(0, numBlocks);

                auto map = [&](const tbb::blocked_range<int>& range) {
                    // Allocate memory for a small image block to be rendered by the current thread
                    ImageBlock block(Vector2i(NORI_BLOCK_SIZE), camera->getReconstructionFilter());
                    ImageBlock normal_block(Vector2i(NORI_BLOCK_SIZE), camera->getReconstructionFilter());
                    ImageBlock texture_block(Vector2i(NORI_BLOCK_SIZE), camera->getReconstructionFilter());
                    ImageBlock depth_block(Vector2i(NORI_BLOCK_SIZE), camera->getReconstructionFilter());
                    ImageBlock visibility_block(Vector2i(NORI_BLOCK_SIZE), camera->getReconstructionFilter());

                    for (int i = range.begin(); i < range.end(); ++i) {
                        // Request an image block from the block generator
                        blockGenerator.next(block);

                        normal_block.setOffset(block.getOffset());
                        texture_block.setOffset(block.getOffset());
                        depth_block.setOffset(block.getOffset());
                        visibility_block.setOffset(block.getOffset());

                        normal_block.setSize(block.getSize());
                        texture_block.setSize(block.getSize());
                        depth_block.setSize(block.getSize());
                        visibility_block.setSize(block.getSize());

                        normal_block.setBlockId(block.getBlockId());
                        texture_block.setBlockId(block.getBlockId());
                        depth_block.setBlockId(block.getBlockId());
                        visibility_block.setBlockId(block.getBlockId());


                        // Get block id to continue using the same sampler
                        auto blockId = block.getBlockId();
                        if (k == 0) { // Initialize the sampler for the first sample
                            std::unique_ptr<Sampler> sampler(m_scene->getSampler()->clone());
                            sampler->prepare(block);
                            samplers.at(blockId) = std::move(sampler);
                        }

                        // Render all contained pixels
                        renderBlock(m_scene, samplers.at(blockId).get(), block, normal_block, texture_block, depth_block, visibility_block);

                        // The image block has been processed. Now add it to the "big" block that represents the entire image
                        m_block_single_sample.put(block);
                        normal_block_single.put(normal_block);
                        texture_block_single.put(texture_block);
                        depth_block_single.put(depth_block);
                        visibility_block_single.put(visibility_block);

                    }

                };

                /// Uncomment the following line for single threaded rendering
                //map(range);

                /// Default: parallel rendering
                tbb::parallel_for(range, map);

                ComputeVariacneAndUpdate(m_block, m_block_single_sample, m_block_variance);
                ComputeVariacneAndUpdate2(normal_FVB, normal_block_single);
                ComputeVariacneAndUpdate2(texture_FVB, texture_block_single);
                ComputeVariacneAndUpdate2(depth_FVB, depth_block_single);
                ComputeVariacneAndUpdate2(visibility_FVB, visibility_block_single);

                blockGenerator.reset();
            }

            cout << "done. (took " << timer.elapsedString() << ")" << endl;


            SaveFeatureAndVariance(m_block, m_block, m_block_variance, numSamples, "output", false);
            SaveFeatureAndVariance(normal_FVB.block1, normal_FVB.block2, normal_FVB.variance_block, numSamples, "normal", true);
            SaveFeatureAndVariance(texture_FVB.block1, texture_FVB.block2, texture_FVB.variance_block, numSamples, "texture", true);
            SaveFeatureAndVariance(depth_FVB.block1, depth_FVB.block2, depth_FVB.variance_block, numSamples, "depth", true);
            SaveFeatureAndVariance(visibility_FVB.block1, visibility_FVB.block2, visibility_FVB.variance_block, numSamples, "visibility", true);




            delete m_scene;
            m_scene = nullptr;

            m_render_status = 3;
            });

    }
    else {
        delete root;
    }

}


NORI_NAMESPACE_END