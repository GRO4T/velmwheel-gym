from unittest.mock import patch

import pytest

from velmwheel_rl.common import get_model_save_path_and_tb_log_name


@pytest.mark.parametrize(
    "algorithm, model_path, mock_listdir_out, expected",
    [
        (
            "PPO",
            None,
            [".gitkeep", "5", "4", "3"],
            ("./models/ppo/6", "PPO_6"),
        ),
        (
            "DDPG",
            None,
            ["5", "4", "3"],
            ("./models/ddpg/6", "DDPG_6"),
        ),
        (
            "TD3",
            None,
            ["3", "2", "1"],
            ("./models/td3/4", "TD3_4"),
        ),
        (
            "PPO",
            "./models/ppo/6/ppo_20000_steps.zip",
            [".gitkeep", "6", "5", "4", "3"],
            ("./models/ppo/6", "PPO_6"),
        ),
    ],
)
def test_get_model_save_path_and_tb_log_name(
    algorithm, model_path, mock_listdir_out, expected
):
    with patch("os.listdir", return_value=mock_listdir_out):
        assert get_model_save_path_and_tb_log_name(algorithm, model_path) == expected
