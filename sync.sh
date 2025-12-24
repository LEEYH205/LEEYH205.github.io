#!/bin/bash
# 블로그 저장소 동기화 스크립트
# 최신 변경사항을 가져옵니다

set -e

echo "=========================================="
echo "블로그 저장소 동기화 시작"
echo "=========================================="

# Git 저장소인지 확인
if [ ! -d .git ]; then
    echo "❌ 오류: Git 저장소가 아닙니다."
    exit 1
fi

# 현재 브랜치 확인
CURRENT_BRANCH=$(git branch --show-current)
echo "📍 현재 브랜치: $CURRENT_BRANCH"

# 변경사항이 있으면 경고
if [ -n "$(git status --porcelain)" ]; then
    echo "⚠️  경고: 로컬에 커밋되지 않은 변경사항이 있습니다."
    echo "   다음 파일들이 변경되었습니다:"
    git status --short
    echo ""
    read -p "계속하시겠습니까? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ 취소되었습니다."
        exit 1
    fi
fi

# 원격 저장소에서 최신 변경사항 가져오기
echo ""
echo "📥 Git 변경사항 가져오는 중..."
git fetch origin

# 원격과 로컬의 차이 확인
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u} 2>/dev/null || echo "")

if [ -z "$REMOTE" ]; then
    echo "⚠️  원격 브랜치를 찾을 수 없습니다. 처음 푸시가 필요할 수 있습니다."
else
    if [ "$LOCAL" != "$REMOTE" ]; then
        echo "🔄 원격 저장소에 새로운 커밋이 있습니다."
        echo "   로컬: $LOCAL"
        echo "   원격: $REMOTE"
        echo ""
        echo "📥 병합 중..."
        git pull origin "$CURRENT_BRANCH" || {
            echo "❌ 병합 충돌이 발생했습니다. 수동으로 해결해주세요."
            exit 1
        }
        echo "✅ 병합 완료"
    else
        echo "✅ 이미 최신 상태입니다."
    fi
fi

# LFS 파일 가져오기
echo ""
echo "📥 LFS 파일 가져오는 중..."
if git lfs pull 2>/dev/null; then
    echo "✅ LFS 파일 동기화 완료"
else
    echo "⚠️  LFS pull 실패 (예산 초과일 수 있음). 계속 진행합니다."
fi

echo ""
echo "=========================================="
echo "✅ 동기화 완료!"
echo "=========================================="

