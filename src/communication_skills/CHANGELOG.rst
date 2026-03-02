^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package communication_skills
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2025-12-01)
------------------
* add version number
* update issue templates
* Create issue template fro new skill
* Contributors: Séverin Lemaignan

1.4.0 (2025-09-10)
------------------
* ament_auto_package now requires USE_SCOPED_HEADER_INSTALL_DIR
* Contributors: Séverin Lemaignan

1.3.2 (2025-06-25)
------------------
* refine skills parameters documentation
* Contributors: Séverin Lemaignan

1.3.1 (2025-06-25)
------------------
* adjust the YAML descriptions to ensure correct rendering of the documentation
* Contributors: Séverin Lemaignan

1.3.0 (2025-06-25)
------------------
* rewrite manifests in YAML with more detailled documentation
* doc
* Contributors: Séverin Lemaignan

1.2.0 (2025-06-18)
------------------
* {skillint -> archlint}
* enable tests, including checking manifest validity
* Contributors: Séverin Lemaignan

1.1.0 (2025-06-12)
------------------
* add the /skill namespace to the default_interface_path
* move here the actual skills definitions
* Contributors: Séverin Lemaignan, ferrangebelli

1.0.0 (2025-04-30)
------------------
* fix documentation
* fix Ask doc
* fix Chat doc
* align Ask documentation with the chatbot_msgs/Dialogue one
* update Chat and Action to new Dialogue role
* Chat and Ask can specify the initial robot utterance
* clearer field naming in Chat result msg
* add Chat; fix documentation
* change Ask queries to be a dictionary
* Contributors: Luka Juricic

0.3.0 (2025-02-25)
------------------
* remove ask and say feedback; use person_id field instead of user_id
* Contributors: Luka Juricic

0.2.0 (2025-02-13)
------------------
* [doc] minor
* rename pkg to communication_skills
* add Ask and Say skills
* Initial commit
* Contributors: Luka Juricic, Séverin Lemaignan
