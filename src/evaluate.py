#!/usr/bin/env python3
import click
import cv2
import numpy as np
from pathlib import Path

from visual_landing_provider.utils.common import parse_yaml_params 
from visual_landing_provider.visual_pipeline_wrapper import VisualPipelineWrapper



class VisualEvaluation:
    def __init__(self, params: dict, source_path: str, visualize: bool):
        """ Class for evaluation of visual pipeline using images and depth maps from directory.

        Parameters
        ----------
        params : dict
            Parameters for the pipeline loaded from the config.yaml file
        source_path : str
            Path to the folder with images to evaluate
        visualize : bool
            Flag to visualize the results
        """
        self.base_path = Path(__file__).resolve().parents[1]
        self.visualize = visualize

        self.pipeline = VisualPipelineWrapper(base_path=self.base_path, config=params, visualize=visualize)

        self.images_to_eval = sorted(Path(source_path).glob('*.png'))

    def run_over_images(self):
        """ Run the pipeline over all images in the source directory. """

        for i, img_path in enumerate(self.images_to_eval):
            image = cv2.cvtColor(cv2.imread(str(img_path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            depth = np.load(open(str(img_path).replace('_img.png', '_depth.npy'), 'rb'))

            results = self.pipeline.process(image, depth=depth)

            if results['pose'] is None:
                print(f'Error: {results["error"]}')
            else:
                print(f'Pose: {results["pose"]}')

            if self.visualize and results['image'] is not None:
                cv2.imshow('image', cv2.cvtColor(results['image'], cv2.COLOR_RGB2BGR))
                cv2.waitKey(0)


@click.command()
@click.option('--source-path', required=True)
@click.option('--visualize', is_flag=True)
def main(source_path, visualize):
    params = parse_yaml_params()

    ve = VisualEvaluation(params=params, source_path=source_path, visualize=visualize)
    ve.run_over_images()

if __name__ == '__main__':
    main()
