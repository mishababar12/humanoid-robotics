# /sp.implement - Execute Implementation Tasks

## Description
Executes the implementation of a feature following the defined tasks. This command guides the actual coding, testing, and integration work while maintaining SDD compliance.

## Usage
```
/sp.implement <feature-name>
```

## Parameters
- `feature-name` (required): Name of the feature to implement

## Process
1. Reviews specification, plan, and task list
2. Executes tasks in proper dependency order
3. Creates/updates code files as specified
4. Implements proper error handling and validation
5. Adds necessary tests for implemented functionality
6. Creates Prompt History Records (PHRs) for each significant change
7. Validates implementation against acceptance criteria
8. Updates documentation as needed

## Output Files
- Modified/created source code files
- Test files for implemented functionality
- PHR records in `history/prompts/[feature-name]/`
- Updated documentation files

## SDD Compliance Check
- [ ] Implementation matches specification exactly
- [ ] Code follows established architecture plan
- [ ] All tasks completed as defined
- [ ] Tests added and passing
- [ ] Documentation updated
- [ ] Error handling implemented properly