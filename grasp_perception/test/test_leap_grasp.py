import os
from glob import glob
import numpy as np


def main():
    grasp_path = '/home/peter/Downloads/output/valid_leap/succ_collect/**/tabletop_ur10e/**.npy'
    grasp_sample_path = list(set(os.path.dirname(path) for path in glob(grasp_path, recursive=True)))

    for i in range(100):
        grasp_samples = []
        for path in glob(grasp_sample_path[i] + "/**.npy", recursive=True):
            grasp_samples.append(np.load(path, allow_pickle=True).item())

        print(f"object name and scale: {grasp_samples[0]['scene_path']}")

        if 'grasp_qpos' in grasp_samples[0]:
            robot_pose = np.vstack([sample['grasp_qpos'] for sample in grasp_samples])
        elif 'robot_pose' in grasp_samples[0]:
            robot_pose = np.vstack([sample['robot_pose'][0, :, 1] for sample in grasp_samples])
        else:
            raise ValueError("Robot pose is not stored in recognizable format.")


if __name__ == '__main__':
    main()
