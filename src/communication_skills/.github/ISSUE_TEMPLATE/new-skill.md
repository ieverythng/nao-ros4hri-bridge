---
name: New skill
about: Template to suggest a new skill definition
title: "[new skill] <name of your skill>"
labels: new skill
assignees: ''

---

## New skill: <type here the skill name>

## Use-case

Describe here briefly why you want to add this skill and why it is different
from other existing skills.

## Skill manifest

```yaml
id: <skill id>
description: |
     Write here the documentation of your skill
interface: <action|service|topic>
default_interface_path: /skill/<skill id>
datatype: communication_skills/<action|service|topic>/<MessageDefinition>
functional_domains:
  - communication
parameters:
  in:
      - name: <param_name>
        type: <datatype>
        required: <true|false>
        default: <value (only if required=false)>
        description: "<parameter description>"
      - <... additional parameters>
   out:
      - name: <param_name>
        type: <datatype>
        required: <true|false>
        default: <value (only if required=false)>
        description: "<parameter description>"
      - <... additional parameters>
```
