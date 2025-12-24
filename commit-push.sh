#!/bin/bash
# 블로그 글 커밋 및 푸시 스크립트
# 자동으로 pull 후 커밋/푸시합니다

set -e

echo "=========================================="
echo "블로그 글 커밋 및 푸시"
echo "=========================================="

# Git 저장소인지 확인
if [ ! -d .git ]; then
    echo "❌ 오류: Git 저장소가 아닙니다."
    exit 1
fi

# 현재 브랜치 확인
CURRENT_BRANCH=$(git branch --show-current)
echo "📍 현재 브랜치: $CURRENT_BRANCH"

# 변경사항 확인
if [ -z "$(git status --porcelain)" ]; then
    echo "ℹ️  커밋할 변경사항이 없습니다."
    exit 0
fi

echo ""
echo "📝 변경된 파일:"
git status --short
echo ""

# 커밋 메시지 입력
if [ -z "$1" ]; then
    read -p "💬 커밋 메시지를 입력하세요: " COMMIT_MSG
    if [ -z "$COMMIT_MSG" ]; then
        echo "❌ 커밋 메시지가 비어있습니다. 취소되었습니다."
        exit 1
    fi
else
    COMMIT_MSG="$1"
fi

# 원격 저장소에서 최신 변경사항 확인
echo ""
echo "📥 원격 저장소 확인 중..."
git fetch origin

LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u} 2>/dev/null || echo "")

if [ -n "$REMOTE" ] && [ "$LOCAL" != "$REMOTE" ]; then
    echo "🔄 원격 저장소에 새로운 커밋이 있습니다."
    echo "   로컬: $LOCAL"
    echo "   원격: $REMOTE"
    echo ""
    echo "📥 자동으로 pull 합니다..."
    git pull origin "$CURRENT_BRANCH" || {
        echo "❌ 병합 충돌이 발생했습니다."
        echo "   수동으로 해결한 후 다시 시도해주세요."
        exit 1
    }
    echo "✅ 병합 완료"
else
    echo "✅ 로컬이 최신 상태입니다."
fi

# 변경사항 스테이징
echo ""
echo "📦 변경사항 스테이징 중..."
git add -A

# 커밋
echo "💾 커밋 중..."
git commit -m "$COMMIT_MSG"

# 푸시
echo ""
echo "🚀 푸시 중..."
git push origin "$CURRENT_BRANCH"

# LFS 파일 푸시 확인
if git lfs ls-files | grep -q "."; then
    echo ""
    echo "📤 LFS 파일 푸시 확인 중..."
    # LFS 파일은 일반 push에 포함되지만 확인 메시지 출력
    echo "✅ LFS 파일 포함됨"
fi

echo ""
echo "=========================================="
echo "✅ 커밋 및 푸시 완료!"
echo "=========================================="
echo "🌐 블로그: https://leeyh205.github.io"

