#!/bin/bash

# In ra trạng thái Git trước khi commit
echo "Checking git status..."
git status

# Thêm tất cả thay đổi
echo "Adding changes..."
git add .

# Tạo commit với thời gian hiện tại
COMMIT_MSG="update: $(date '+%Y-%m-%d %H:%M:%S')"
echo "Commit with message: '$COMMIT_MSG'"
git commit -m "$COMMIT_MSG"

# Đẩy lên nhánh hiện tại
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo "Pushing to branch: $CURRENT_BRANCH"
git push origin "$CURRENT_BRANCH"

echo "Commit & Push completed!"
