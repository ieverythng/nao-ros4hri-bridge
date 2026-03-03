from pathlib import Path

from nao_chatbot.skill_catalog import build_skill_catalog_text


def test_build_skill_catalog_text_from_manifest(tmp_path: Path, monkeypatch) -> None:
    pkg_dir = tmp_path / "src" / "my_skill_pkg"
    pkg_dir.mkdir(parents=True)
    package_xml = pkg_dir / "package.xml"
    package_xml.write_text(
        """
<package format="3">
  <name>my_skill_pkg</name>
  <export>
    <skill content-type="yaml">
      id: wave
      description: |
        Wave to the user.
      default_interface_path: /skill/wave
      datatype: my_pkg/action/Wave
      parameters:
        in:
          - name: speed
            type: float
    </skill>
  </export>
</package>
""".strip(),
        encoding="utf-8",
    )

    monkeypatch.chdir(tmp_path)
    text, descriptors = build_skill_catalog_text(
        package_names=["my_skill_pkg"],
        max_entries=16,
        max_chars=2000,
    )

    assert len(descriptors) == 1
    assert descriptors[0].skill_id == "wave"
    assert "/skill/wave" in text
    assert "speed" in text
