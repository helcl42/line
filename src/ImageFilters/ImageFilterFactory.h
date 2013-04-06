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

        std::vector<Kernel<T>* > kernels;
        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerXMatrix));
        kernels.push_back(new NonSeparableKernel<T>(3, 3, kerYMatrix));

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
        imageFilterBatch->addItem(gaussFIlter);        
        
        return imageFilterBatch;
    }
};

#endif	/* IMAGEFILTERFACTORY_H */

