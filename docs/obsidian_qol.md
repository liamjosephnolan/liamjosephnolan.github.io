---
layout: page
title: Obsidian QOL Improvement
description: Improving my personal Obsidian configuration to improve my quality of life
hide_description: true
date: 16 Oct 2024
---
Okay, let's talk about Obsidian, the note-taking sensation that has been sweeping the nation (for the last 5 years—I’m just late to the game).

While it seems like a lot of tech bloggers/vloggers use Obsidian just to make more content about Obsidian, it is a legitimately cool tool with a lot of nice features.

I've played with it enough now that I am pretty committed to using it for everyday documentation and organization. There is a lot to like about it:

**Pros:**
- Open Source
- Free
- Uses Markdown
- Lightweight (until you add a couple of hundred plugins)
- Massive community, which means loads of documentation

The only real con I have found is that if you want to sync it using their native service, you have to pay $4/month.

I don't want to do that, so let's figure out an alternative.

There are two main things I want:
- My markdown notes backed up in the cloud
- Syncing between my laptop (Linux) and my phone (iOS)

I could use iCloud, but it’s messy and kind of a pain in Linux with rsync automation in Cron. Instead, I am just going to host my files in a private GitHub repo. It’s almost all markdown text with a few images here and there, so storage shouldn't be an issue.

I created a private repo and cloned it into my Obsidian folder. Once I pushed all my local files, I used the Git plugin to automatically push regularly. Pretty simple, but the phone is a bit trickier.

Did you know there is a Linux terminal app for iPhone? I didn’t, and it’s honestly great. iSH is free, and you can even SSH into it. 5 stars!

I mostly followed this guide [here](https://forum.obsidian.md/t/mobile-sync-with-git-on-ios-for-free-using-ish/20861).

High-level steps are:
- `mkdir` with the same name as the Obsidian repo
- Mount my local vault folder into this new directory
- Clone my repo into this folder

The Obsidian iOS app has all the same plugins, so once I configured the Git settings for my iPhone, I shouldn't need to worry about it. I have been testing it a bit, and it works really well so far.

Going to start messing around with Templater for some climbing data tracking next.
