/* 
 * File:   ImageFilterFactory.h
 * Author: lubos
 *
 * Created on April 5, 2013, 4:57 PM
 */

#ifndef IMAGEFILTERFACTORY_H
#define	IMAGEFILTERFACTORY_H

#include "Frequency/FFTImageFilterBatch.h"
#include "Frequency/FFTImageFilterBatchItem.h"
#include "Kernel/Kernel.h"
#include "Kernel/NonSeparableKernel.h"

template <class T>
class ImageFilterFactory
{
public:
    static FFTImageFilterBatch<T>* createBatch()
    {
        float kerXMatrix[] = {
            -1, -2, -1,
             0,  0,  0,
             1,  2,  1
        };

        float kerYMatrix[] = {
            -1, 0, 1,
            -2, 0, 2,
            -1, 0, 1
        };
        
        
        float ker1Matrix[] = {
            -2, -1,  0,
            -1,  0,  1,
             0,  1,  2
        };
        
        float ker2Matrix[] = {
             0,  1,  2,
            -1,  0,  1,
            -2, -1,  0
        };
        
        float ker3Matrix[] = {
             1,  2,  1,
             0,  0,  0,
            -1, -2, -1
        };
        
        float ker4Matrix[] = {
            2,  1,  0,
            1,  0, -1,
            0, -1, -2
        };
        
        float ker5Matrix[] = {
            1,  0, -1,
            2,  0, -2,
            1,  0, -1
        };
        
        float ker6Matrix[] = {
            0, -1, -2,
            1,  0, -1,
            2,  1,  0
        };
        
        std::vector<Kernel<T>* > kernels;
        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerXMatrix));
        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerYMatrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker1Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker2Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker3Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker4Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker5Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, ker6Matrix));

        FFTImageFilterBatchItem<T>* edgeFilter = new FFTImageFilterBatchItem<T>(kernels);


        float gaussKer[] = {
            0.029637889913828982, 0.036720652003741625, 0.03943950161493383, 0.036720652003741625, 0.029637889913828982,
            0.036720652003741625, 0.04549602847909662, 0.048864619519598224, 0.04549602847909662, 0.036720652003741625,
            0.03943950161493383, 0.048864619519598224, 0.052482625860236644, 0.048864619519598224, 0.03943950161493383,
            0.036720652003741625, 0.04549602847909662, 0.048864619519598224, 0.04549602847909662, 0.036720652003741625,
            0.029637889913828982, 0.036720652003741625, 0.03943950161493383, 0.036720652003741625, 0.029637889913828982

        };
                

        Kernel<T>* gaussianKernel = new NonSeparableKernel<T>(5, 5, gaussKer);

        FFTImageFilterBatchItem<T>* gaussFIlter = new FFTImageFilterBatchItem<T>(gaussianKernel);

        FFTImageFilterBatch<T>* imageFilterBatch = new FFTImageFilterBatch<T>();
        imageFilterBatch->addItem(edgeFilter);
        //imageFilterBatch->addItem(gaussFIlter);        
        
        return imageFilterBatch;
    }
    
//    static FFTImageFilterBatch<T>* createBatch()
//    {
//        float kerXMatrix[] = {
//            -1, -2, -1,
//             0,  0,  0,
//             1,  2,  1
//        };
//
//        float kerYMatrix[] = {
//            -1, 0, 1,
//            -2, 0, 2,
//            -1, 0, 1
//        };
//
//        float kerYXMatrix[] = {
//             0,  1, 2,
//            -1,  0, 1,
//            -2, -1, 0
//        };
//        
//        float kerXYMatrix[] = {
//            -2, -1, 0,
//            -1,  0, 1,
//             0,  1, 2
//        };
//        
//        float kerYX2Matrix[] = {
//             1,  2,  1,
//             0,  0,  0,
//            -1, -2, -1
//        };
//        
//        float kerXY2Matrix[] = {
//            2,  1,  0,
//            1,  0, -1,
//            0, -1, -2
//        };
//        
//        std::vector<Kernel<T>* > kernels;
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerXMatrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerYMatrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerYXMatrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerXYMatrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerYX2Matrix));
//        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerXY2Matrix));
//
//        FFTImageFilterBatchItem<T>* edgeFilter = new FFTImageFilterBatchItem<T>(kernels);
//
//
//        float gaussKer[] = {
//            0.029637889913828982, 0.036720652003741625, 0.03943950161493383, 0.036720652003741625, 0.029637889913828982,
//            0.036720652003741625, 0.04549602847909662, 0.048864619519598224, 0.04549602847909662, 0.036720652003741625,
//            0.03943950161493383, 0.048864619519598224, 0.052482625860236644, 0.048864619519598224, 0.03943950161493383,
//            0.036720652003741625, 0.04549602847909662, 0.048864619519598224, 0.04549602847909662, 0.036720652003741625,
//            0.029637889913828982, 0.036720652003741625, 0.03943950161493383, 0.036720652003741625, 0.029637889913828982
//
//        };
//
//        Kernel<T>* gaussianKernel = new NonSeparableKernel<T>(5, 5, gaussKer);
//
//        FFTImageFilterBatchItem<T>* gaussFIlter = new FFTImageFilterBatchItem<T>(gaussianKernel);
//
//        FFTImageFilterBatch<T>* imageFilterBatch = new FFTImageFilterBatch<T>();
//        imageFilterBatch->addItem(edgeFilter);
//        imageFilterBatch->addItem(gaussFIlter);        
//        
//        return imageFilterBatch;
//    }
};

#endif	/* IMAGEFILTERFACTORY_H */


