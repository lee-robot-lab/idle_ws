from pathlib import Path


def test_sim_pickplace_launch_exposes_model_xml_argument():
    text = Path("src/idle_launch/launch/sim_pickplace.launch.py").read_text()
    assert '"model_xml"' in text
    assert 'LaunchConfiguration("model_xml")' in text


def test_sim_pickplace_launch_does_not_hardcode_old_home():
    text = Path("src/idle_launch/launch/sim_pickplace.launch.py").read_text()
    assert "/home/su/idle_ws" not in text
