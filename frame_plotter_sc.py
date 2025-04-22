from plotter import PlotCamera
import numpy as np

CamPlotter = PlotCamera(snapFrame= 1)

framesOrg        = np.load('data/cyclegan/turbo/frames_generated/data_org.npy')  # shape: (num_frames, 256, 256, 3)
framesWinterFake = np.load('data/cyclegan/turbo/frames_generated/data_winter.npy')  # shape: (num_frames, 256, 256, 3)
framesSummerFake = np.load('data/cyclegan/turbo/frames_generated/data_summerfromwinter.npy')  # shape: (num_frames, 256, 256, 3)
framesFakeSat    = np.load('data/cyclegan/turbo/frames_generated/data_fakesat.npy')  # shape: (num_frames, 256, 256, 3)

for i in range(len(framesOrg)):

    CamPlotter.snapNow(
                    (framesOrg[i]                 , 'UAV Org Camera'                         ),
                    (framesWinterFake[i]             , 'Generated Fake Winter from Org Camera'  ),
                    (framesSummerFake[i]  , 'Generated Fake Summer from Fake Winter' ),
                    (framesFakeSat[i]  , 'Generated Fake Satellite from Fake Summer' )

                    )