# Contributing to ssl_ros_bridge

First off, thanks for taking the time to contribute! ❤️

All types of contributions are encouraged and valued. See the [Table of Contents](#table-of-contents) for different ways to help and details about how this project handles them. Please make sure to read the relevant section before making your contribution. It will make it a lot easier for us maintainers and smooth out the experience for all involved. The community looks forward to your contributions. 🎉

## Table of Contents

<!-- - [Code of Conduct](#code-of-conduct) -->
- [I Have a Question](#i-have-a-question)
- [I Want To Contribute](#i-want-to-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Enhancements](#suggesting-enhancements)
- [Styleguides](#styleguides)

<!-- TODO
## Code of Conduct

This project and everyone participating in it is governed by the
[CONTRIBUTING.md Code of Conduct](blob/master/CODE_OF_CONDUCT.md).
By participating, you are expected to uphold this code. Please report unacceptable behavior
to <>. -->

## I Have a Question

> If you want to ask a question, we assume that you have read the available [documentation](README.md).

Before you ask a question, it is best to search for existing [Issues](https://github.com/SSL-A-Team/ssl_ros_bridge/issues) or [Discussions](https://github.com/SSL-A-Team/ssl_ros_bridge/discussions) that might help you. In case you have found a suitable issue/topic and still need clarification, you can write your question in the existing issue/topic.

If you don't find what you're looking for there, you can start a [new question topic](https://github.com/SSL-A-Team/ssl_ros_bridge/discussions/new?category=q-a).

If it turns out your question highlights a new bug in our software, we'll create an issue from your Q&A post.

For quicker / shorter questions, you can also tag "@[A-Team] Matt Barulic" in the [Open Source channel](https://discord.com/channels/465455923415220234/467643497122496543) in the league Discord server.

## I Want To Contribute

> ### Legal Notice 
> When contributing to this project, you must agree that you have authored 100% of the content, that you have the necessary rights to the content and that the content you contribute may be provided under the project license.

### Reporting Bugs

#### Before Submitting a Bug Report

A good bug report shouldn't leave others needing to chase you up for more information. Therefore, we ask you to investigate carefully, collect information and describe the issue in detail in your report. Please complete the following steps in advance to help us fix any potential bug as fast as possible.

- Make sure that you are using the latest version.
- Determine if your bug is really a bug and not an error on your side e.g. using incompatible environment components/versions (Make sure that you have read the [documentation](README.md). If you are looking for support, you might want to check [this section](#i-have-a-question)).
- To see if other users have experienced (and potentially already solved) the same issue you are having, check if there is not already a bug report existing for your bug or error in the [bug tracker](issues?q=label%3Abug).
- Collect information about the bug:
    - Any relevant traces or log output
    - OS, Platform and Version (Windows, Linux, macOS, x86, ARM)
    - ROS version being used
    - Can you reliably reproduce the issue? And can you also reproduce it with older versions?

#### How Do I Submit a Good Bug Report?

We use GitHub issues to track bugs and errors. If you run into an issue with the project:

- Open an [Issue](https://github.com/SSL-A-Team/ssl_ros_bridge/issues/new). (Since we can't be sure at this point whether it is a bug or not, we ask you not to talk about a bug yet and not to label the issue.)
- Explain the behavior you would expect and the actual behavior.
- Please provide as much context as possible and describe the *reproduction steps* that someone else can follow to recreate the issue on their own. This usually includes your code. For good bug reports you should isolate the problem and create a reduced test case.
- Provide the information you collected in the previous section.

Once it's filed:

- The project team will label the issue accordingly.
- A team member will try to reproduce the issue with your provided steps. If there are no reproduction steps or no obvious way to reproduce the issue, the team will ask you for those steps and mark the issue as `needs-repro`. Bugs with the `needs-repro` tag will not be addressed until they are reproduced.
- If the team is able to reproduce the issue, it will be marked `needs-fix`, as well as possibly other tags (such as `critical`), and the issue will be left to be [implemented by someone](#your-first-code-contribution).
- The project team may convert the issue to a discussions topic if it is a question or otherwise does not capture an actionable issue.

### Suggesting Enhancements

This section guides you through submitting an enhancement suggestion for CONTRIBUTING.md, **including completely new features and minor improvements to existing functionality**. Following these guidelines will help maintainers and the community to understand your suggestion and find related suggestions.

#### Before Submitting an Enhancement

- Make sure that you are using the latest version.
- Read the [documentation](README.md) carefully and find out if the functionality is already covered, maybe by an individual configuration.
- Perform a [search](https://github.com/SSL-A-Team/ssl_ros_bridge/issues) to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.
- Find out whether your idea fits with the scope and aims of the project. It's up to you to make a strong case to convince the project's developers of the merits of this feature. Keep in mind that we want features that will be useful to the majority of our users and not just a small subset. If you're just targeting a minority of users, consider writing an add-on/plugin library.
- Feel free to have early discussions about your suggestions via discussions or on Discord as presented in [this section](#i-have-a-question).

#### How Do I Submit a Good Enhancement Suggestion?

Enhancement suggestions are tracked as [GitHub issues](https://github.com/SSL-A-Team/ssl_ros_bridge/issues).

- Use a **clear and descriptive title** for the issue to identify the suggestion.
- Provide a **step-by-step description of the suggested enhancement** in as many details as possible.
- **Describe the current behavior** and **explain which behavior you expected to see instead** and why. At this point you can also tell which alternatives do not work for you.
- You may want to **include screenshots and animated GIFs** which help you demonstrate the steps or point out the part which the suggestion is related to.
- **Explain why this enhancement would be useful** to most ssl_ros_bridge users. You may also want to point out the other projects that solved it better and which could serve as inspiration.

## Styleguides

This project follows the [ROS 2 style guides](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

### Commit Messages

Generally, commit messages should be a phrase starting with the verb capturing what the commit changes. A good rule of thumb is to write a sentence like "This commit fixes/changes/improves/etc..." and use everything after "This commit" as your commit message.

Tag relevant issues in your commit messages.

**Good commit message**: "Fixes #1 by reordering network calls."

**Bad commit message**: "Network calls no longer broken."

## Attribution

This guide is based on **contributing.md**. [Make your own](https://contributing.md/)!
